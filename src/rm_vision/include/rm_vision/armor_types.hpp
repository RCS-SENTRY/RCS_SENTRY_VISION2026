// =============================================================================
// armor_types.hpp — 轻量级装甲板类型定义（不依赖 Eigen / OpenCV）
// =============================================================================
// 从旧架构 armor.hpp 中剥离的纯类型定义。
// 仅保留 YOLO 检测器输出所需的字段，剔除所有 3D/PnPL/Tracker 字段。
// armor_properties 映射表原封不动保留。
// =============================================================================
#ifndef RM_VISION__ARMOR_TYPES_HPP
#define RM_VISION__ARMOR_TYPES_HPP

#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

namespace rm_vision
{

// ---- 颜色枚举 ----
enum class ArmorColor : uint8_t
{
  RED = 0,
  BLUE = 1,
  EXTINGUISH = 2,
  PURPLE = 3,
  NONE = 255
};

// ---- 装甲板尺寸枚举 ----
enum class ArmorSize : uint8_t
{
  SMALL = 0,
  BIG = 1
};

// ---- 编号枚举 ----
enum class ArmorName : uint8_t
{
  ONE = 0,
  TWO,
  THREE,
  FOUR,
  FIVE,
  SENTRY,
  OUTPOST,
  BASE,
  NOT_ARMOR
};

// ---- 可读标签 ----
const std::vector<std::string> ARMOR_COLOR_NAMES = {"red", "blue", "extinguish", "purple"};
const std::vector<std::string> ARMOR_NAME_NAMES = {
  "one", "two", "three", "four", "five", "sentry", "outpost", "base", "not_armor"};
const std::vector<std::string> ARMOR_SIZE_NAMES = {"small", "big"};

// =============================================================================
// YOLO class_id → 属性映射表
// 原封不动保留自旧架构 armor.hpp 中的 armor_properties
// 格式: (Color, Name, Size)
// =============================================================================
// clang-format off
const std::vector<std::tuple<ArmorColor, ArmorName, ArmorSize>> ARMOR_PROPERTIES = {
  {ArmorColor::BLUE,       ArmorName::SENTRY,     ArmorSize::SMALL},
  {ArmorColor::RED,        ArmorName::SENTRY,     ArmorSize::SMALL},
  {ArmorColor::EXTINGUISH, ArmorName::SENTRY,     ArmorSize::SMALL},

  {ArmorColor::BLUE,       ArmorName::ONE,        ArmorSize::SMALL},
  {ArmorColor::RED,        ArmorName::ONE,        ArmorSize::SMALL},
  {ArmorColor::EXTINGUISH, ArmorName::ONE,        ArmorSize::SMALL},

  {ArmorColor::BLUE,       ArmorName::TWO,        ArmorSize::SMALL},
  {ArmorColor::RED,        ArmorName::TWO,        ArmorSize::SMALL},
  {ArmorColor::EXTINGUISH, ArmorName::TWO,        ArmorSize::SMALL},

  {ArmorColor::BLUE,       ArmorName::THREE,      ArmorSize::SMALL},
  {ArmorColor::RED,        ArmorName::THREE,      ArmorSize::SMALL},
  {ArmorColor::EXTINGUISH, ArmorName::THREE,      ArmorSize::SMALL},

  {ArmorColor::BLUE,       ArmorName::FOUR,       ArmorSize::SMALL},
  {ArmorColor::RED,        ArmorName::FOUR,       ArmorSize::SMALL},
  {ArmorColor::EXTINGUISH, ArmorName::FOUR,       ArmorSize::SMALL},

  {ArmorColor::BLUE,       ArmorName::FIVE,       ArmorSize::SMALL},
  {ArmorColor::RED,        ArmorName::FIVE,       ArmorSize::SMALL},
  {ArmorColor::EXTINGUISH, ArmorName::FIVE,       ArmorSize::SMALL},

  {ArmorColor::BLUE,       ArmorName::OUTPOST,    ArmorSize::SMALL},
  {ArmorColor::RED,        ArmorName::OUTPOST,    ArmorSize::SMALL},
  {ArmorColor::EXTINGUISH, ArmorName::OUTPOST,    ArmorSize::SMALL},

  {ArmorColor::BLUE,       ArmorName::BASE,       ArmorSize::BIG},
  {ArmorColor::RED,        ArmorName::BASE,       ArmorSize::BIG},
  {ArmorColor::EXTINGUISH, ArmorName::BASE,       ArmorSize::BIG},
  {ArmorColor::PURPLE,     ArmorName::BASE,       ArmorSize::BIG},

  {ArmorColor::BLUE,       ArmorName::BASE,       ArmorSize::SMALL},
  {ArmorColor::RED,        ArmorName::BASE,       ArmorSize::SMALL},
  {ArmorColor::EXTINGUISH, ArmorName::BASE,       ArmorSize::SMALL},
  {ArmorColor::PURPLE,     ArmorName::BASE,       ArmorSize::SMALL},

  {ArmorColor::BLUE,       ArmorName::THREE,      ArmorSize::BIG},
  {ArmorColor::RED,        ArmorName::THREE,      ArmorSize::BIG},
  {ArmorColor::EXTINGUISH, ArmorName::THREE,      ArmorSize::BIG},

  {ArmorColor::BLUE,       ArmorName::FOUR,       ArmorSize::BIG},
  {ArmorColor::RED,        ArmorName::FOUR,       ArmorSize::BIG},
  {ArmorColor::EXTINGUISH, ArmorName::FOUR,       ArmorSize::BIG},

  {ArmorColor::BLUE,       ArmorName::FIVE,       ArmorSize::BIG},
  {ArmorColor::RED,        ArmorName::FIVE,       ArmorSize::BIG},
  {ArmorColor::EXTINGUISH, ArmorName::FIVE,       ArmorSize::BIG},
};
// clang-format on

// ---- 生成人类可读标签 ----
inline std::string make_armor_label(int class_id)
{
  if (class_id < 0 || class_id >= static_cast<int>(ARMOR_PROPERTIES.size())) {
    return "unknown";
  }
  const auto & [color, name, size] = ARMOR_PROPERTIES[class_id];
  return ARMOR_COLOR_NAMES[static_cast<int>(color)] + "_" +
         ARMOR_NAME_NAMES[static_cast<int>(name)] + "_" +
         ARMOR_SIZE_NAMES[static_cast<int>(size)];
}

// ---- 从 class_id 解析属性 ----
inline bool parse_class_id(
  int class_id, ArmorColor & out_color, ArmorName & out_name, ArmorSize & out_size)
{
  if (class_id < 0 || class_id >= static_cast<int>(ARMOR_PROPERTIES.size())) {
    out_color = ArmorColor::NONE;
    out_name  = ArmorName::NOT_ARMOR;
    out_size  = ArmorSize::SMALL;
    return false;
  }
  const auto & [c, n, s] = ARMOR_PROPERTIES[class_id];
  out_color = c;
  out_name  = n;
  out_size  = s;
  return true;
}

}  // namespace rm_vision

#endif  // RM_VISION__ARMOR_TYPES_HPP