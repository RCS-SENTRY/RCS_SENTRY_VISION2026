// =============================================================================
// hik_camera_node.cpp — 海康工业相机 ROS 2 驱动 (v2 — 极致优化版)
// =============================================================================
// [P0-FIX] 强制硬件 PixelFormat=BGR8，CPU 零 Bayer 解码
// [P0-FIX] camera_mutex_ 保护句柄，capture 线程 join 后才销毁
// [P0-FIX] stamp 紧贴 GetImageBuffer 成功返回
// [P1-FIX] on_parameter_update 实时写 exposure/gain 到硬件
// =============================================================================
#include "rm_hik_driver/hik_camera_node.hpp"

#include <algorithm>
#include <cstring>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace rm_hik_driver
{

// =============================================================================
// 构造函数
// =============================================================================
HikCameraNode::HikCameraNode(const rclcpp::NodeOptions & options)
: Node("rm_hik_driver", options)
{
  // ---- 声明参数 ----
  this->declare_parameter<std::string>("camera_name", "hik_camera");
  this->declare_parameter<int>("exposure_time", 2000);
  this->declare_parameter<double>("gain", 10.0);
  this->declare_parameter<bool>("is_auto_exposure", false);
  this->declare_parameter<int>("target_fps", 150);
  this->declare_parameter<std::string>("frame_id", "camera_optical_frame");
  this->declare_parameter<std::string>(
    "camera_info_url",
    "package://rm_hik_driver/config/hik_camera_info.yaml");

  // ---- 读取参数 ----
  camera_name_      = this->get_parameter("camera_name").as_string();
  exposure_time_    = this->get_parameter("exposure_time").as_int();
  gain_             = this->get_parameter("gain").as_double();
  is_auto_exposure_ = this->get_parameter("is_auto_exposure").as_bool();
  target_fps_       = this->get_parameter("target_fps").as_int();
  frame_id_         = this->get_parameter("frame_id").as_string();

  // ---- SDK 版本 ----
  unsigned int sdk_ver = MV_CC_GetSDKVersion();
  RCLCPP_INFO(get_logger(), "HikRobot MVS SDK Version: %d.%d.%d.%d",
    (sdk_ver >> 24) & 0xFF, (sdk_ver >> 16) & 0xFF,
    (sdk_ver >> 8) & 0xFF, sdk_ver & 0xFF);

  // ---- Publisher: SensorDataQoS (Best Effort, Keep Last 1) ----
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/camera/image_raw", rclcpp::SensorDataQoS());

  // ---- CameraInfo Publisher (与 Image 同步发布) ----
  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/camera_info", rclcpp::SensorDataQoS());

  // ---- CameraInfo Manager (从 YAML 加载标定参数) ----
  const auto camera_info_url = this->get_parameter("camera_info_url").as_string();
  camera_info_mgr_ = std::make_unique<camera_info_manager::CameraInfoManager>(
    this, camera_name_, camera_info_url);
  if (camera_info_mgr_->isCalibrated()) {
    RCLCPP_INFO(get_logger(),
      "Camera calibrated! fx=%.2f fy=%.2f cx=%.2f cy=%.2f, distortion=%s",
      camera_info_mgr_->getCameraInfo().k[0],
      camera_info_mgr_->getCameraInfo().k[4],
      camera_info_mgr_->getCameraInfo().k[2],
      camera_info_mgr_->getCameraInfo().k[5],
      camera_info_mgr_->getCameraInfo().distortion_model.c_str());
  } else {
    RCLCPP_WARN(get_logger(),
      "Camera NOT calibrated! PnP accuracy will be severely degraded. "
      "Check camera_info_url: %s", camera_info_url.c_str());
  }

  // ---- 预分配消息对象 ----
  preallocated_msg_ = std::make_shared<sensor_msgs::msg::Image>();
  preallocated_msg_->header.frame_id = frame_id_;
  preallocated_msg_->encoding = "bgr8";
  preallocated_msg_->is_bigendian = 0;

  // ---- 首次打开相机 ----
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!open_camera()) {
      RCLCPP_ERROR(get_logger(),
        "Failed to open camera on startup. Watchdog will retry...");
    }
  }

  // ---- 启动取流线程 ----
  capture_running_.store(true);
  capture_thread_ = std::thread(&HikCameraNode::capture_loop, this);

  // ---- 启动看门狗线程 ----
  watchdog_running_.store(true);
  watchdog_thread_ = std::thread([this]() {
    RCLCPP_INFO(get_logger(), "Watchdog thread started.");
    while (watchdog_running_.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(2));
      if (!camera_connected_.load()) {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        RCLCPP_WARN(get_logger(), "Camera disconnected. Attempting reconnect...");
        close_camera();
        if (open_camera()) {
          RCLCPP_INFO(get_logger(), "Camera reconnected successfully.");
        } else {
          RCLCPP_WARN(get_logger(), "Reconnect failed. Will retry in 2s...");
        }
      }
    }
    RCLCPP_INFO(get_logger(), "Watchdog thread stopped.");
  });

  // ---- [P1-FIX] 参数热更新回调 ----
  param_callback_ = this->add_on_set_parameters_callback(
    std::bind(&HikCameraNode::on_parameter_update, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "rm_hik_driver v2 initialized. "
    "exposure=%dus, gain=%.1f, auto_exposure=%s, fps=%d, frame_id=%s",
    exposure_time_, gain_, is_auto_exposure_ ? "ON" : "OFF",
    target_fps_, frame_id_.c_str());
}

// =============================================================================
// 析构函数 — 严格顺序: 停线程 → 销毁句柄
// =============================================================================
HikCameraNode::~HikCameraNode()
{
  // 1. 停止取流线程
  capture_running_.store(false);
  camera_cv_.notify_all();
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }

  // 2. 停止看门狗
  watchdog_running_.store(false);
  if (watchdog_thread_.joinable()) {
    watchdog_thread_.join();
  }

  // 3. 此时只剩一个线程，安全销毁 SDK 句柄
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    close_camera();
  }

  RCLCPP_INFO(get_logger(), "rm_hik_driver v2 destructed.");
}

// =============================================================================
// 枚举并打开相机设备
// ★ 调用方必须持有 camera_mutex_
// =============================================================================
bool HikCameraNode::open_camera()
{
  MV_CC_DEVICE_INFO_LIST device_list;
  memset(&device_list, 0, sizeof(device_list));

  unsigned int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    RCLCPP_WARN(get_logger(), "MV_CC_EnumDevices failed: 0x%08X", ret);
    camera_connected_.store(false);
    return false;
  }

  if (device_list.nDeviceNum == 0) {
    memset(&device_list, 0, sizeof(device_list));
    ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
    if (device_list.nDeviceNum == 0) {
      RCLCPP_WARN(get_logger(), "No camera devices found!");
      camera_connected_.store(false);
      return false;
    }
  }

  RCLCPP_INFO(get_logger(), "Found %u camera device(s).", device_list.nDeviceNum);

  MV_CC_DEVICE_INFO * dev_info = device_list.pDeviceInfo[0];
  if (dev_info->nTLayerType == MV_USB_DEVICE) {
    RCLCPP_INFO(get_logger(), "USB Camera: VID=0x%04X, PID=0x%04X",
      dev_info->SpecialInfo.stUsb3VInfo.idVendor,
      dev_info->SpecialInfo.stUsb3VInfo.idProduct);
  } else if (dev_info->nTLayerType == MV_GIGE_DEVICE) {
    RCLCPP_INFO(get_logger(), "GigE Camera: IP=%d.%d.%d.%d",
      (dev_info->SpecialInfo.stGigEInfo.nCurrentIp >> 24) & 0xFF,
      (dev_info->SpecialInfo.stGigEInfo.nCurrentIp >> 16) & 0xFF,
      (dev_info->SpecialInfo.stGigEInfo.nCurrentIp >> 8) & 0xFF,
      dev_info->SpecialInfo.stGigEInfo.nCurrentIp & 0xFF);
  }

  ret = MV_CC_CreateHandle(&camera_handle_, dev_info);
  if (ret != MV_OK) {
    RCLCPP_ERROR(get_logger(), "MV_CC_CreateHandle failed: 0x%08X", ret);
    camera_connected_.store(false);
    return false;
  }

  ret = MV_CC_OpenDevice(camera_handle_);
  if (ret != MV_OK) {
    RCLCPP_ERROR(get_logger(), "MV_CC_OpenDevice failed: 0x%08X", ret);
    MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
    camera_connected_.store(false);
    return false;
  }

  apply_camera_settings();

  ret = MV_CC_SetFrameRate(camera_handle_, static_cast<float>(target_fps_));
  if (ret != MV_OK) {
    RCLCPP_WARN(get_logger(), "MV_CC_SetFrameRate(%d) failed: 0x%08X", target_fps_, ret);
  }

  ret = MV_CC_StartGrabbing(camera_handle_);
  if (ret != MV_OK) {
    RCLCPP_ERROR(get_logger(), "MV_CC_StartGrabbing failed: 0x%08X", ret);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
    camera_connected_.store(false);
    return false;
  }

  camera_connected_.store(true);
  camera_cv_.notify_all();
  return true;
}

// =============================================================================
// 关闭相机
// ★ 调用方必须持有 camera_mutex_
// =============================================================================
void HikCameraNode::close_camera()
{
  camera_connected_.store(false);

  if (camera_handle_ == nullptr) return;

  MV_CC_StopGrabbing(camera_handle_);
  MV_CC_CloseDevice(camera_handle_);
  MV_CC_DestroyHandle(camera_handle_);
  camera_handle_ = nullptr;
}

// =============================================================================
// 下发相机参数
// ★ 调用方必须持有 camera_mutex_
// ★ [P0-FIX] 强制硬件 BGR8 输出
// =============================================================================
void HikCameraNode::apply_camera_settings()
{
  if (camera_handle_ == nullptr) return;

  // ★★★ 强制硬件输出 BGR8 — 消灭 CPU 侧 Bayer 转换 ★★★
  set_enum_param("PixelFormat", PixelType_Gvsp_BGR8_Packed);

  // 白平衡
  set_enum_param("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);

  if (is_auto_exposure_) {
    set_enum_param("ExposureAuto", MV_EXPOSURE_AUTO_MODE_CONTINUOUS);
    set_enum_param("GainAuto", MV_GAIN_MODE_CONTINUOUS);
  } else {
    set_enum_param("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    set_enum_param("GainAuto", MV_GAIN_MODE_OFF);

    int clamped_exp = std::clamp(exposure_time_, kExposureMin, kExposureMax);
    if (clamped_exp != exposure_time_) {
      RCLCPP_WARN(get_logger(),
        "Exposure %d clamped to [%d, %d] -> %d",
        exposure_time_, kExposureMin, kExposureMax, clamped_exp);
      exposure_time_ = clamped_exp;
    }
    set_float_param("ExposureTime", static_cast<double>(exposure_time_));

    MVCC_FLOATVALUE gain_range;
    memset(&gain_range, 0, sizeof(gain_range));
    unsigned int ret = MV_CC_GetFloatValue(camera_handle_, "Gain", &gain_range);
    if (ret == MV_OK) {
      double clamped = std::clamp(gain_,
        static_cast<double>(gain_range.fMin),
        static_cast<double>(gain_range.fMax));
      if (std::abs(clamped - gain_) > 1e-6) {
        RCLCPP_WARN(get_logger(),
          "Gain %.3f out of [%.3f, %.3f], clamped to %.3f",
          gain_, gain_range.fMin, gain_range.fMax, clamped);
      }
      set_float_param("Gain", clamped);
    } else {
      RCLCPP_WARN(get_logger(),
        "MV_CC_GetFloatValue(\"Gain\") failed: 0x%08X, fallback 16.0", ret);
      set_float_param("Gain", 16.0);
    }
  }
}

// =============================================================================
// 预分配 Image 消息
// =============================================================================
void HikCameraNode::preallocate_image_buffer(uint32_t width, uint32_t height)
{
  std::lock_guard<std::mutex> lock(msg_mutex_);
  if (width == expected_width_ && height == expected_height_) return;

  preallocated_msg_->width  = width;
  preallocated_msg_->height = height;
  preallocated_msg_->step   = width * 3;
  preallocated_msg_->data.resize(static_cast<size_t>(width) * height * 3);

  expected_width_  = width;
  expected_height_ = height;

  RCLCPP_INFO(get_logger(),
    "Preallocated image buffer: %ux%u = %zu bytes",
    width, height, preallocated_msg_->data.size());
}

// =============================================================================
// 取流主循环
// =============================================================================
void HikCameraNode::capture_loop()
{
  RCLCPP_INFO(get_logger(), "Capture thread started.");

  while (capture_running_.load()) {
    if (!camera_connected_.load()) {
      std::unique_lock<std::mutex> lock(camera_mutex_);
      camera_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
        return camera_connected_.load() || !capture_running_.load();
      });
      continue;
    }

    MV_FRAME_OUT raw;
    memset(&raw, 0, sizeof(raw));

    unsigned int ret;
    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      if (camera_handle_ == nullptr || !camera_connected_.load()) continue;
      ret = MV_CC_GetImageBuffer(camera_handle_, &raw, 100);
    }

    if (ret != MV_OK) {
      if (ret == 0x80000001) continue;
      RCLCPP_WARN(get_logger(), "MV_CC_GetImageBuffer failed: 0x%08X", ret);
      camera_connected_.store(false);
      camera_cv_.notify_all();
      continue;
    }

    // ★★★ [P0-FIX] 时间戳紧贴 GetImageBuffer 成功返回 ★★★
    auto stamp = this->now();

    uint32_t w = raw.stFrameInfo.nWidth;
    uint32_t h = raw.stFrameInfo.nHeight;
    size_t data_len = raw.stFrameInfo.nFrameLen;

    preallocate_image_buffer(w, h);

    // ★★★ [P0-FIX] 一次 memcpy: SDK buffer → preallocated msg ★★★
    {
      std::lock_guard<std::mutex> lock(msg_mutex_);
      auto & dst = preallocated_msg_->data;

      if (data_len <= dst.size()) {
        memcpy(dst.data(), raw.pBufAddr, data_len);
      } else {
        dst.resize(data_len);
        memcpy(dst.data(), raw.pBufAddr, data_len);
      }

      preallocated_msg_->header.stamp = stamp;
    }

    // 释放 SDK 缓冲区
    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      if (camera_handle_) {
        MV_CC_FreeImageBuffer(camera_handle_, &raw);
      }
    }

    // 发布 Image（shared_ptr 引用计数 +1，数据零拷贝）
    image_pub_->publish(*preallocated_msg_);

    // 同步发布 CameraInfo（与 Image 共享相同 stamp 和 frame_id）
    if (camera_info_pub_->get_subscription_count() > 0) {
      auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(
        camera_info_mgr_->getCameraInfo());
      ci->header.stamp    = stamp;
      ci->header.frame_id = frame_id_;
      ci->width  = w;
      ci->height = h;
      camera_info_pub_->publish(*ci);
    }

    frame_count_++;
    if (frame_count_.load() % kLogIntervalFrames == 0) {
      RCLCPP_INFO(get_logger(),
        "[HikDriver] frame#=%u  resolution=%ux%u",
        frame_count_.load(), w, h);
    }
  }

  RCLCPP_INFO(get_logger(), "Capture thread stopped.");
}

// =============================================================================
// [P1-FIX] 参数热更新回调
// =============================================================================
rcl_interfaces::msg::SetParametersResult HikCameraNode::on_parameter_update(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : params) {
    if (param.get_name() == "exposure_time" && !is_auto_exposure_) {
      int val = static_cast<int>(std::clamp(param.as_int(),
        static_cast<int64_t>(kExposureMin), static_cast<int64_t>(kExposureMax)));
      exposure_time_ = val;

      std::lock_guard<std::mutex> lock(camera_mutex_);
      if (camera_handle_) {
        set_float_param("ExposureTime", static_cast<double>(val));
        RCLCPP_INFO(get_logger(), "[HotUpdate] exposure_time -> %dus", val);
      }
    }
    else if (param.get_name() == "gain" && !is_auto_exposure_) {
      double val = param.as_double();

      std::lock_guard<std::mutex> lock(camera_mutex_);
      if (camera_handle_) {
        MVCC_FLOATVALUE gain_range;
        memset(&gain_range, 0, sizeof(gain_range));
        unsigned int ret = MV_CC_GetFloatValue(camera_handle_, "Gain", &gain_range);
        if (ret == MV_OK) {
          val = std::clamp(val,
            static_cast<double>(gain_range.fMin),
            static_cast<double>(gain_range.fMax));
        }
        gain_ = val;
        set_float_param("Gain", val);
        RCLCPP_INFO(get_logger(), "[HotUpdate] gain -> %.3f", val);
      }
    }
    else if (param.get_name() == "is_auto_exposure") {
      is_auto_exposure_ = param.as_bool();

      std::lock_guard<std::mutex> lock(camera_mutex_);
      if (camera_handle_) {
        if (is_auto_exposure_) {
          set_enum_param("ExposureAuto", MV_EXPOSURE_AUTO_MODE_CONTINUOUS);
          set_enum_param("GainAuto", MV_GAIN_MODE_CONTINUOUS);
        } else {
          set_enum_param("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
          set_enum_param("GainAuto", MV_GAIN_MODE_OFF);
          set_float_param("ExposureTime", static_cast<double>(exposure_time_));
          set_float_param("Gain", gain_);
        }
        RCLCPP_INFO(get_logger(), "[HotUpdate] auto_exposure -> %s",
          is_auto_exposure_ ? "ON" : "OFF");
      }
    }
  }

  return result;
}

// =============================================================================
// SDK 参数辅助（调用方持有 camera_mutex_）
// =============================================================================
void HikCameraNode::set_float_param(const std::string & name, double value)
{
  if (!camera_handle_) return;
  unsigned int ret = MV_CC_SetFloatValue(camera_handle_, name.c_str(), value);
  if (ret != MV_OK) {
    RCLCPP_WARN(get_logger(),
      "MV_CC_SetFloatValue(\"%s\", %.3f) failed: 0x%08X", name.c_str(), value, ret);
  }
}

void HikCameraNode::set_enum_param(const std::string & name, unsigned int value)
{
  if (!camera_handle_) return;
  unsigned int ret = MV_CC_SetEnumValue(camera_handle_, name.c_str(), value);
  if (ret != MV_OK) {
    RCLCPP_WARN(get_logger(),
      "MV_CC_SetEnumValue(\"%s\", %u) failed: 0x%08X", name.c_str(), value, ret);
  }
}

}  // namespace rm_hik_driver

// =============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_hik_driver::HikCameraNode>());
  rclcpp::shutdown();
  return 0;
}