#!/usr/bin/env python3

"""
pcd2pgm.py
==========

把 Point-LIO / small_gicp 使用的 3D PCD 地图，转换成 Nav2 需要的 2D PGM/YAML。

职责边界:
  - PCD: 给 small_gicp / rm_global_localization 做三维配准定位
  - PGM/YAML: 给 Nav2 map_server / costmap 做二维占据栅格导航

设计目标:
  1. 不做“整张点云直接拍扁”。
  2. 先估计主地面，再把主地面对齐到 +Z，兼容倾斜安装 LiDAR / 轻微地图残余倾角。
  3. 地面 / 已观测区域和障碍高度切片分开建模:
     - known/free: 已观测区域
     - occupied: 指定高度切片的障碍
     - unknown: 未观测区域
  4. 只依赖 NumPy + PyYAML，尽量避免现场 Python 环境兼容性炸点。
"""

from __future__ import annotations

import argparse
import math
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np
import yaml


FREE_VALUE = 254
UNKNOWN_VALUE = 205
OCCUPIED_VALUE = 0
MAX_MAP_CELLS = 200_000_000
PCD_TYPE_MAP = {
    ("F", 4): np.float32,
    ("F", 8): np.float64,
    ("I", 1): np.int8,
    ("I", 2): np.int16,
    ("I", 4): np.int32,
    ("I", 8): np.int64,
    ("U", 1): np.uint8,
    ("U", 2): np.uint16,
    ("U", 4): np.uint32,
    ("U", 8): np.uint64,
}


@dataclass
class GridBounds:
    min_x: float
    min_y: float
    max_x: float
    max_y: float
    width: int
    height: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a 3D PCD map into Nav2-compatible .pgm + .yaml"
    )
    parser.add_argument("pcd", type=Path, help="输入静态 PCD 地图")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path.cwd(),
        help="输出目录，默认当前目录",
    )
    parser.add_argument(
        "--map-name",
        "--output-name",
        type=str,
        dest="map_name",
        default=None,
        help="输出地图名，不带后缀；默认取 PCD 文件名",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.05,
        help="地图分辨率，单位 m/cell",
    )
    parser.add_argument(
        "--meters-per-unit",
        type=float,
        default=0.0,
        help="输入点云坐标换算到米的比例；>0 时强制使用，0 表示自动判断 mm/m",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.05,
        help="体素降采样大小，单位 m；<=0 表示关闭",
    )
    parser.add_argument(
        "--ground-band-min",
        type=float,
        default=-0.20,
        help="已观测区域高度下界，相对地面，单位 m",
    )
    parser.add_argument(
        "--ground-band-max",
        type=float,
        default=0.12,
        help="已观测区域高度上界，相对地面，单位 m",
    )
    parser.add_argument(
        "--obstacle-min-z",
        "--z-min",
        type=float,
        dest="obstacle_min_z",
        default=0.15,
        help="障碍高度切片下界，相对地面，单位 m",
    )
    parser.add_argument(
        "--obstacle-max-z",
        "--z-max",
        type=float,
        dest="obstacle_max_z",
        default=1.80,
        help="障碍高度切片上界，相对地面，单位 m",
    )
    parser.add_argument(
        "--align-ground",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="是否先估计主地面并对齐到 +Z [默认关闭: Point-LIO 输出已经是 gravity-aligned]",
    )
    parser.add_argument(
        "--ground-distance-threshold",
        type=float,
        default=0.03,
        help="地面 RANSAC 内点阈值，单位 m",
    )
    parser.add_argument(
        "--ground-num-iterations",
        type=int,
        default=800,
        help="地面 RANSAC 迭代次数",
    )
    parser.add_argument(
        "--radius-outlier-nb-points",
        type=int,
        default=0,
        help="半径离群点去除最少邻居数；<=0 表示关闭",
    )
    parser.add_argument(
        "--radius-outlier-radius",
        type=float,
        default=0.15,
        help="半径离群点去除搜索半径，单位 m",
    )
    parser.add_argument(
        "--min-component-size",
        type=int,
        default=8,
        help="2D 栅格最小连通域像素数，小于该值的障碍噪点会被删掉",
    )
    parser.add_argument(
        "--opening-kernel",
        type=int,
        default=0,
        help="形态学开运算核大小；<=0 表示关闭",
    )
    parser.add_argument(
        "--closing-kernel",
        type=int,
        default=3,
        help="形态学闭运算核大小；<=0 表示关闭",
    )
    parser.add_argument(
        "--known-dilate-kernel",
        type=int,
        default=3,
        help="已观测区域膨胀核大小；用于连接地面稀疏采样",
    )
    parser.add_argument(
        "--padding",
        type=float,
        default=0.50,
        help="XY 边界额外留白，单位 m",
    )
    return parser.parse_args()


def normalize_kernel_size(kernel_size: int) -> int:
    if kernel_size <= 0:
        return 0
    if kernel_size % 2 == 0:
        kernel_size += 1
    return kernel_size


def read_pcd_points(path: Path) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(f"PCD 文件不存在: {path}")

    header: Dict[str, str] = {}
    with path.open("rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError("PCD 文件缺少 DATA 头")
            line_str = line.decode("ascii", errors="ignore").strip()
            if not line_str or line_str.startswith("#"):
                continue
            parts = line_str.split(maxsplit=1)
            key = parts[0].upper()
            value = parts[1] if len(parts) > 1 else ""
            header[key] = value
            if key == "DATA":
                data_offset = f.tell()
                break

        fields = header.get("FIELDS", "").split()
        sizes = [int(item) for item in header.get("SIZE", "").split()]
        types = header.get("TYPE", "").split()
        counts_raw = header.get("COUNT", "")
        counts = [int(item) for item in counts_raw.split()] if counts_raw else [1] * len(fields)
        points_count = int(header.get("POINTS", "0") or "0")
        width = int(header.get("WIDTH", "0") or "0")
        height = int(header.get("HEIGHT", "0") or "0")
        data_kind = header["DATA"].lower()

        if len(fields) != len(sizes) or len(fields) != len(types) or len(fields) != len(counts):
            raise RuntimeError("PCD 头中的 FIELDS/SIZE/TYPE/COUNT 数量不一致")
        if not points_count:
            points_count = width * height
        if points_count <= 0:
            raise RuntimeError("PCD 头中没有有效的 POINTS 数量")

        if "x" not in fields or "y" not in fields or "z" not in fields:
            raise RuntimeError("PCD 必须至少包含 x/y/z 字段")

        if data_kind == "ascii":
            f.seek(data_offset)
            usecols = [fields.index("x"), fields.index("y"), fields.index("z")]
            points = np.loadtxt(f, dtype=np.float64, usecols=usecols)
            return np.atleast_2d(points)

        if data_kind != "binary":
            raise RuntimeError(f"暂不支持 PCD DATA 模式: {data_kind}")

        dtype_fields = []
        for field, size, type_code, count in zip(fields, sizes, types, counts):
            np_dtype = PCD_TYPE_MAP.get((type_code, size))
            if np_dtype is None:
                raise RuntimeError(f"不支持的 PCD 字段类型: TYPE={type_code} SIZE={size}")
            if count == 1:
                dtype_fields.append((field, np_dtype))
            else:
                dtype_fields.append((field, np_dtype, (count,)))
        dtype = np.dtype(dtype_fields)

        f.seek(data_offset)
        raw = f.read()
        points_all = np.frombuffer(raw, dtype=dtype, count=points_count)
        points = np.column_stack(
            [
                points_all["x"].astype(np.float64),
                points_all["y"].astype(np.float64),
                points_all["z"].astype(np.float64),
            ]
        )
        return points


def filter_finite_points(points: np.ndarray) -> np.ndarray:
    mask = np.isfinite(points).all(axis=1)
    filtered = points[mask]
    if filtered.size == 0:
        raise RuntimeError("点云中没有有效的有限点")
    return filtered


def choose_unit_scale(points: np.ndarray, meters_per_unit: float) -> Tuple[float, str]:
    if meters_per_unit > 0.0:
        return meters_per_unit, f"explicit meters_per_unit={meters_per_unit:.6f}"

    q_low = np.quantile(points, 0.01, axis=0)
    q_high = np.quantile(points, 0.99, axis=0)
    robust_span = q_high - q_low
    robust_extent = np.maximum(np.abs(q_low), np.abs(q_high))

    looks_like_millimeters = (
        np.max(robust_extent[:2]) > 100.0
        or np.max(robust_span[:2]) > 150.0
        or robust_extent[2] > 20.0
    )
    if looks_like_millimeters:
        return 0.001, "auto-detected millimeter-scale input"
    return 1.0, "auto-detected meter-scale input"


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if voxel_size <= 0.0:
        return points

    voxel_indices = np.floor(points / voxel_size).astype(np.int64)
    unique_indices, inverse = np.unique(voxel_indices, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    downsampled = np.empty((len(unique_indices), 3), dtype=np.float64)
    for axis in range(3):
        downsampled[:, axis] = np.bincount(inverse, weights=points[:, axis]) / counts
    return downsampled


def radius_outlier_filter(points: np.ndarray, radius: float, min_neighbors: int) -> np.ndarray:
    if radius <= 0.0 or min_neighbors <= 0:
        return points

    cell_size = radius
    voxel_indices = np.floor(points / cell_size).astype(np.int64)
    buckets: Dict[Tuple[int, int, int], List[int]] = defaultdict(list)
    for index, voxel in enumerate(voxel_indices):
        buckets[tuple(voxel.tolist())].append(index)

    radius_sq = radius * radius
    keep_mask = np.zeros(len(points), dtype=bool)
    neighbor_offsets = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
    ]

    for index, voxel in enumerate(voxel_indices):
        point = points[index]
        neighbor_count = 0
        vx, vy, vz = voxel.tolist()
        for dx, dy, dz in neighbor_offsets:
            candidates = buckets.get((vx + dx, vy + dy, vz + dz), ())
            for candidate_index in candidates:
                diff = points[candidate_index] - point
                if np.dot(diff, diff) <= radius_sq:
                    neighbor_count += 1
                    if neighbor_count >= min_neighbors:
                        keep_mask[index] = True
                        break
            if keep_mask[index]:
                break

    filtered = points[keep_mask]
    if filtered.size == 0:
        raise RuntimeError("半径离群点去除后点云为空，请放宽参数")
    return filtered


def fit_plane_from_three_points(samples: np.ndarray) -> Tuple[np.ndarray, float] | None:
    p1, p2, p3 = samples
    normal = np.cross(p2 - p1, p3 - p1)
    norm = np.linalg.norm(normal)
    if norm < 1e-9:
        return None
    normal = normal / norm
    d = -float(np.dot(normal, p1))
    if normal[2] < 0.0:
        normal = -normal
        d = -d
    return normal, d


def refine_plane(inliers: np.ndarray) -> Tuple[np.ndarray, float]:
    centroid = np.mean(inliers, axis=0)
    centered = inliers - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    if normal[2] < 0.0:
        normal = -normal
    d = -float(np.dot(normal, centroid))
    return normal, d


def rotation_matrix_from_vectors(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source = source / np.linalg.norm(source)
    target = target / np.linalg.norm(target)

    cross = np.cross(source, target)
    norm = np.linalg.norm(cross)
    dot = float(np.clip(np.dot(source, target), -1.0, 1.0))

    if norm < 1e-9:
        if dot > 0.0:
            return np.eye(3)
        axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(source[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        axis = axis - np.dot(axis, source) * source
        axis = axis / np.linalg.norm(axis)
        return axis_angle_to_matrix(axis, math.pi)

    axis = cross / norm
    angle = math.atan2(norm, dot)
    return axis_angle_to_matrix(axis, angle)


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis.tolist()
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=np.float64,
    )


def align_ground(points: np.ndarray, args: argparse.Namespace) -> Tuple[np.ndarray, np.ndarray]:
    if not args.align_ground:
        return points, np.eye(4)

    rng = np.random.default_rng(42)
    z_threshold = np.quantile(points[:, 2], 0.40)
    candidate_points = points[points[:, 2] <= z_threshold]
    if len(candidate_points) < 3:
        candidate_points = points

    evaluation_points = candidate_points
    if len(evaluation_points) > 50000:
        evaluation_points = evaluation_points[
            rng.choice(len(evaluation_points), size=50000, replace=False)
        ]

    best_score = -1.0
    best_plane: Tuple[np.ndarray, float] | None = None
    distance_threshold = args.ground_distance_threshold

    for _ in range(args.ground_num_iterations):
        sample_indices = rng.choice(len(candidate_points), size=3, replace=False)
        plane = fit_plane_from_three_points(candidate_points[sample_indices])
        if plane is None:
            continue
        normal, d = plane
        alignment_weight = 0.5 + 0.5 * abs(normal[2])
        if alignment_weight < 0.65:
            continue
        distances = np.abs(evaluation_points @ normal + d)
        inlier_count = int(np.count_nonzero(distances <= distance_threshold))
        score = inlier_count * alignment_weight
        if score > best_score:
            best_score = score
            best_plane = plane

    if best_plane is None:
        raise RuntimeError("主地面 RANSAC 失败，无法估计地面")

    initial_normal, initial_d = best_plane
    all_distances = np.abs(points @ initial_normal + initial_d)
    inliers = points[all_distances <= distance_threshold]
    if len(inliers) < 3:
        raise RuntimeError("主地面内点过少，无法完成地面对齐")

    refined_normal, _ = refine_plane(inliers)
    rotation = rotation_matrix_from_vectors(refined_normal, np.array([0.0, 0.0, 1.0]))
    rotated_points = points @ rotation.T
    rotated_inliers = inliers @ rotation.T
    ground_z = float(np.median(rotated_inliers[:, 2]))
    aligned_points = rotated_points.copy()
    aligned_points[:, 2] -= ground_z

    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.array([0.0, 0.0, -ground_z], dtype=np.float64)
    return aligned_points, transform


def compute_bounds(points_sets: Iterable[np.ndarray], resolution: float, padding: float) -> GridBounds:
    valid_sets = [pts for pts in points_sets if pts.size > 0]
    if not valid_sets:
        raise RuntimeError("没有可用于投影的点，请放宽高度切片或滤波参数")

    stacked = np.vstack(valid_sets)
    min_x = float(np.min(stacked[:, 0]) - padding)
    min_y = float(np.min(stacked[:, 1]) - padding)
    max_x = float(np.max(stacked[:, 0]) + padding)
    max_y = float(np.max(stacked[:, 1]) + padding)

    width = max(1, int(math.ceil((max_x - min_x) / resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / resolution)))
    cell_count = width * height
    if cell_count > MAX_MAP_CELLS:
        raise RuntimeError(
            "投影后的二维地图过大: "
            f"{width} x {height} = {cell_count / 1e6:.1f}M cells. "
            "请检查输入点云单位是否为毫米、是否需要 --meters-per-unit 0.001，"
            "或适当增大 resolution / 减小 padding。"
        )
    return GridBounds(min_x, min_y, max_x, max_y, width, height)


def rasterize(points: np.ndarray, bounds: GridBounds, resolution: float) -> np.ndarray:
    mask = np.zeros((bounds.height, bounds.width), dtype=np.uint8)
    if points.size == 0:
        return mask

    xs = np.floor((points[:, 0] - bounds.min_x) / resolution).astype(np.int32)
    ys = np.floor((bounds.max_y - points[:, 1]) / resolution).astype(np.int32)
    valid = (
        (xs >= 0)
        & (xs < bounds.width)
        & (ys >= 0)
        & (ys < bounds.height)
    )
    mask[ys[valid], xs[valid]] = 255
    return mask


def binary_dilate(mask: np.ndarray, kernel_size: int) -> np.ndarray:
    kernel_size = normalize_kernel_size(kernel_size)
    if kernel_size == 0 or not np.any(mask):
        return mask

    source = mask > 0
    pad = kernel_size // 2
    padded = np.pad(source, pad_width=pad, mode="constant", constant_values=False)
    windows = np.lib.stride_tricks.sliding_window_view(padded, (kernel_size, kernel_size))
    dilated = np.any(windows, axis=(-2, -1))
    return np.where(dilated, 255, 0).astype(np.uint8)


def binary_erode(mask: np.ndarray, kernel_size: int) -> np.ndarray:
    kernel_size = normalize_kernel_size(kernel_size)
    if kernel_size == 0 or not np.any(mask):
        return mask

    source = mask > 0
    pad = kernel_size // 2
    padded = np.pad(source, pad_width=pad, mode="constant", constant_values=False)
    windows = np.lib.stride_tricks.sliding_window_view(padded, (kernel_size, kernel_size))
    eroded = np.all(windows, axis=(-2, -1))
    return np.where(eroded, 255, 0).astype(np.uint8)


def morphology(mask: np.ndarray, op: str, kernel_size: int) -> np.ndarray:
    kernel_size = normalize_kernel_size(kernel_size)
    if kernel_size == 0 or not np.any(mask):
        return mask

    if op == "open":
        return binary_dilate(binary_erode(mask, kernel_size), kernel_size)
    if op == "close":
        return binary_erode(binary_dilate(mask, kernel_size), kernel_size)
    raise ValueError(f"Unsupported morphology op: {op}")


def keep_large_components(mask: np.ndarray, min_component_size: int) -> np.ndarray:
    if min_component_size <= 1 or not np.any(mask):
        return mask

    source = mask > 0
    visited = np.zeros_like(source, dtype=bool)
    filtered = np.zeros_like(source, dtype=bool)
    height, width = source.shape

    for row in range(height):
        for col in range(width):
            if not source[row, col] or visited[row, col]:
                continue

            queue = deque([(row, col)])
            visited[row, col] = True
            component = []

            while queue:
                current_row, current_col = queue.popleft()
                component.append((current_row, current_col))
                for delta_row in (-1, 0, 1):
                    for delta_col in (-1, 0, 1):
                        if delta_row == 0 and delta_col == 0:
                            continue
                        next_row = current_row + delta_row
                        next_col = current_col + delta_col
                        if (
                            next_row < 0
                            or next_row >= height
                            or next_col < 0
                            or next_col >= width
                            or visited[next_row, next_col]
                            or not source[next_row, next_col]
                        ):
                            continue
                        visited[next_row, next_col] = True
                        queue.append((next_row, next_col))

            if len(component) >= min_component_size:
                for current_row, current_col in component:
                    filtered[current_row, current_col] = True

    return np.where(filtered, 255, 0).astype(np.uint8)


def build_map_image(free_mask: np.ndarray, occupied_mask: np.ndarray) -> np.ndarray:
    image = np.full(free_mask.shape, UNKNOWN_VALUE, dtype=np.uint8)
    image[free_mask > 0] = FREE_VALUE
    image[occupied_mask > 0] = OCCUPIED_VALUE
    return image


def save_outputs(
    map_image: np.ndarray,
    bounds: GridBounds,
    resolution: float,
    output_dir: Path,
    map_name: str,
) -> Tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = output_dir / f"{map_name}.pgm"
    yaml_path = output_dir / f"{map_name}.yaml"

    with pgm_path.open("wb") as f:
        header = f"P5\n{map_image.shape[1]} {map_image.shape[0]}\n255\n".encode("ascii")
        f.write(header)
        f.write(map_image.tobytes())

    yaml_data = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": float(resolution),
        "origin": [float(bounds.min_x), float(bounds.min_y), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    with yaml_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(yaml_data, f, allow_unicode=True, sort_keys=False)

    return pgm_path, yaml_path


def _save_debug_pgm(path: Path, mask: np.ndarray) -> None:
    """保存一个二值 mask 为 PGM（白=255, 黑=0），方便用 eog/feh 直接查看。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        header = f"P5\n{mask.shape[1]} {mask.shape[0]}\n255\n".encode("ascii")
        f.write(header)
        f.write(mask.tobytes())


def main() -> None:
    args = parse_args()
    map_name = args.map_name or args.pcd.stem

    if args.obstacle_min_z >= args.obstacle_max_z:
        raise ValueError("--obstacle-min-z 必须小于 --obstacle-max-z")
    if args.ground_band_min >= args.ground_band_max:
        raise ValueError("--ground-band-min 必须小于 --ground-band-max")

    raw_points = read_pcd_points(args.pcd)
    filtered_points = filter_finite_points(raw_points)
    meters_per_unit, unit_reason = choose_unit_scale(filtered_points, args.meters_per_unit)
    filtered_points = filtered_points * meters_per_unit
    filtered_points = voxel_downsample(filtered_points, args.voxel_size)
    filtered_points = radius_outlier_filter(
        filtered_points,
        radius=args.radius_outlier_radius,
        min_neighbors=args.radius_outlier_nb_points,
    )
    aligned_points, transform = align_ground(filtered_points, args)

    z = aligned_points[:, 2]
    known_points = aligned_points[(z >= args.ground_band_min) & (z <= args.obstacle_max_z)]
    ground_points = aligned_points[(z >= args.ground_band_min) & (z <= args.ground_band_max)]
    obstacle_points = aligned_points[(z >= args.obstacle_min_z) & (z <= args.obstacle_max_z)]

    bounds = compute_bounds((known_points, obstacle_points), args.resolution, args.padding)

    free_mask = rasterize(known_points, bounds, args.resolution)
    free_mask = binary_dilate(free_mask, args.known_dilate_kernel)
    free_mask = morphology(free_mask, "close", args.closing_kernel)

    occupied_mask = rasterize(obstacle_points, bounds, args.resolution)
    occupied_mask = keep_large_components(occupied_mask, args.min_component_size)
    occupied_mask = morphology(occupied_mask, "open", args.opening_kernel)
    occupied_mask = morphology(occupied_mask, "close", args.closing_kernel)

    map_image = build_map_image(free_mask, occupied_mask)

    # ---- 保存调试中间产物 ----
    args.output_dir.mkdir(parents=True, exist_ok=True)
    debug_ground = args.output_dir / f"{map_name}_ground_mask.pgm"
    debug_obstacle = args.output_dir / f"{map_name}_obstacle_mask.pgm"
    debug_known = args.output_dir / f"{map_name}_known_mask.pgm"
    _save_debug_pgm(debug_ground, free_mask)
    _save_debug_pgm(debug_obstacle, occupied_mask)
    known_mask = ((map_image == FREE_VALUE) | (map_image == OCCUPIED_VALUE)).astype(np.uint8) * 255
    _save_debug_pgm(debug_known, known_mask)

    pgm_path, yaml_path = save_outputs(
        map_image=map_image,
        bounds=bounds,
        resolution=args.resolution,
        output_dir=args.output_dir,
        map_name=map_name,
    )

    # ---- 统计诊断 ----
    total_pixels = map_image.size
    free_px = int(np.count_nonzero(map_image == FREE_VALUE))
    occ_px = int(np.count_nonzero(map_image == OCCUPIED_VALUE))
    unk_px = int(np.count_nonzero(map_image == UNKNOWN_VALUE))

    print("=" * 60)
    print("pcd2pgm finished")
    print("=" * 60)
    print(f"  input_pcd          : {args.pcd}")
    print(f"  output_pgm         : {pgm_path}")
    print(f"  output_yaml        : {yaml_path}")
    print(f"  resolution         : {args.resolution:.3f} m/cell")
    print(f"  meters_per_unit    : {meters_per_unit:.6f} ({unit_reason})")
    print(f"  align_ground       : {args.align_ground}")
    print(f"  raw_points         : {len(raw_points)}")
    print(f"  filtered_points    : {len(filtered_points)}")
    print(f"  ground_points      : {len(ground_points)}")
    print(f"  obstacle_points    : {len(obstacle_points)}")
    print(f"  map_size           : {bounds.width} x {bounds.height} ({total_pixels} px)")
    print(f"  free  (254)        : {free_px:>8d} px  ({free_px/total_pixels*100:.1f}%)")
    print(f"  occ   (  0)        : {occ_px:>8d} px  ({occ_px/total_pixels*100:.1f}%)")
    print(f"  unk   (205)        : {unk_px:>8d} px  ({unk_px/total_pixels*100:.1f}%)")
    print(f"  debug_ground_mask  : {debug_ground}")
    print(f"  debug_obstacle_mask: {debug_obstacle}")
    print(f"  debug_known_mask   : {debug_known}")
    if free_px + occ_px == 0:
        print("  *** WARNING: 地图全 unknown! 检查 z 切片 / align-ground / 单位 ***")
    elif occ_px == 0:
        print("  *** WARNING: 没有检测到障碍物! 检查 obstacle-min-z / max-z ***")
    print("  note               : PCD → small_gicp; PGM/YAML → Nav2")
    print("=" * 60)


if __name__ == "__main__":
    main()
