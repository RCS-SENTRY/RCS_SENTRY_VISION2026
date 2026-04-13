// =============================================================================
// hik_camera_node.hpp — 海康工业相机 ROS 2 驱动节点 (v2 — 极致优化版)
// =============================================================================
// [P0-FIX] 强制硬件 BGR8 输出，删除 OpenCV/cv_bridge，一次 memcpy
// [P0-FIX] 严格线程同步：capture 线程 join 后才销毁 handle
// [P0-FIX] 时间戳紧贴 GetImageBuffer 成功返回
// [P1-FIX] 参数热更新：exposure_time / gain 实时写入硬件
// =============================================================================
#ifndef RM_HIK_DRIVER__HIK_CAMERA_NODE_HPP
#define RM_HIK_DRIVER__HIK_CAMERA_NODE_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <rclcpp/publisher.hpp>

#include <camera_info_manager/camera_info_manager.hpp>

#include "MvCameraControl.h"

namespace rm_hik_driver
{

class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HikCameraNode() override;

  HikCameraNode(const HikCameraNode &) = delete;
  HikCameraNode & operator=(const HikCameraNode &) = delete;

private:
  // ===========================================================================
  // SDK 操作
  // ===========================================================================
  bool open_camera();
  void close_camera();
  void apply_camera_settings();
  void set_float_param(const std::string & name, double value);
  void set_enum_param(const std::string & name, unsigned int value);

  // 预分配消息缓冲区（根据实际分辨率分配）
  void preallocate_image_buffer(uint32_t width, uint32_t height);

  // ===========================================================================
  // 取流线程
  // ===========================================================================
  void capture_loop();

  // ===========================================================================
  // 参数热更新回调
  // ===========================================================================
  rcl_interfaces::msg::SetParametersResult on_parameter_update(
    const std::vector<rclcpp::Parameter> & params);

  // ===========================================================================
  // 参数
  // ===========================================================================
  std::string camera_name_;
  int exposure_time_;
  double gain_;
  bool is_auto_exposure_;
  int target_fps_;
  std::string frame_id_;

  // 曝光时间 clamp 范围
  static constexpr int kExposureMin = 100;
  static constexpr int kExposureMax = 20000;

  // ===========================================================================
  // SDK 句柄 & 线程同步
  // ===========================================================================
  void * camera_handle_ = nullptr;
  std::mutex camera_mutex_;              // ★ 保护 camera_handle_ 的所有访问
  std::condition_variable camera_cv_;    // ★ 通知 capture 线程相机状态变化

  std::thread capture_thread_;
  std::atomic<bool> capture_running_{false};

  std::thread watchdog_thread_;
  std::atomic<bool> watchdog_running_{false};
  std::atomic<bool> camera_connected_{false};

  // ★ 预分配的 ROS Image 消息（避免每帧 new/resize）
  sensor_msgs::msg::Image::SharedPtr preallocated_msg_;
  std::mutex msg_mutex_;                 // 保护 preallocated_msg_ 的写入
  uint32_t expected_width_ = 0;          // 上次分配时的分辨率
  uint32_t expected_height_ = 0;

  // ===========================================================================
  // ROS 2 接口
  // ===========================================================================
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_mgr_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  std::atomic<uint32_t> frame_count_{0};
  static constexpr uint32_t kLogIntervalFrames = 300;
};

}  // namespace rm_hik_driver

#endif  // RM_HIK_DRIVER__HIK_CAMERA_NODE_HPP