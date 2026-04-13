// =============================================================================
// serial_port.hpp — POSIX 串口封装（ROS 2 友好，无外部依赖）
// =============================================================================
// 提供阻塞写 + 回调驱动读的串口通信能力。
// 内置守护线程：断线自动重连。
// =============================================================================
#ifndef RM_HW_BRIDGE__SERIAL_PORT_HPP_
#define RM_HW_BRIDGE__SERIAL_PORT_HPP_

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/logging.hpp>

namespace rm_hw_bridge
{

class SerialPort
{
public:
  using RxCallback = std::function<void(const uint8_t * data, size_t len)>;

  // ===========================================================================
  /// @brief 构造并打开串口
  /// @param device   设备路径，如 "/dev/ttyUSB0"
  /// @param baudrate 波特率
  /// @param cb       接收回调（在读取线程中调用）
  // ===========================================================================
  SerialPort(
    const std::string & device, int baudrate, RxCallback cb,
    const rclcpp::Logger & logger = rclcpp::get_logger("serial_port"))
  : device_(device),
    baudrate_(baudrate),
    fd_(-1),
    ok_(false),
    quit_(false),
    rx_callback_(std::move(cb)),
    logger_(logger)
  {
    try_open();

    // 守护线程：检测断线并自动重连
    daemon_thread_ = std::thread([this]() {
      while (!quit_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (ok_) continue;

        // 读取线程需要先回收
        if (read_thread_.joinable()) read_thread_.join();

        close_fd();
        try_open();
      }
    });
  }

  ~SerialPort() { shutdown(); }

  // 禁止拷贝
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // ===========================================================================
  /// @brief 向串口写入数据（非阻塞，线程安全）
  /// O_NONBLOCK 确保 ::write() 不会卡死；锁仅保护 fd_ 读写顺序。
  // ===========================================================================
  void write(const uint8_t * data, size_t len)
  {
    int fd;
    {
      std::lock_guard<std::mutex> lock(tx_mutex_);
      fd = fd_;
    }
    if (fd < 0) {
      RCLCPP_WARN(logger_, "Serial fd invalid, drop %zu bytes", len);
      return;
    }
    // 非阻塞写入 — EAGAIN 时直接返回，不重试
    ssize_t n = ::write(fd, data, len);
    if (n == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN(logger_, "Serial write failed: %s", std::strerror(errno));
    }
  }

  /// @brief 便捷重载
  void write(const std::vector<uint8_t> & buf) { write(buf.data(), buf.size()); }

  bool is_ok() const { return ok_; }

  void shutdown()
  {
    quit_ = true;
    if (daemon_thread_.joinable()) daemon_thread_.join();
    if (read_thread_.joinable()) read_thread_.join();
    close_fd();
  }

private:
  std::string device_;
  int baudrate_;
  int fd_;
  std::atomic<bool> ok_;
  std::atomic<bool> quit_;

  std::thread read_thread_;
  std::thread daemon_thread_;
  uint8_t read_buf_[1024];

  RxCallback rx_callback_;
  std::mutex tx_mutex_;
  rclcpp::Logger logger_;

  // -----------------------------------------------------------------------
  // 波特率映射
  // -----------------------------------------------------------------------
  static speed_t to_speed(int baud)
  {
    switch (baud) {
      case 9600:   return B9600;
      case 19200:  return B19200;
      case 38400:  return B38400;
      case 57600:  return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
      case 921600: return B921600;
      default:     return B460800;
    }
  }

  // -----------------------------------------------------------------------
  // 打开串口并启动读取线程
  // -----------------------------------------------------------------------
  void open_port()
  {
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("Cannot open serial: " + device_);
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("tcgetattr failed");
    }

    cfsetospeed(&tty, to_speed(baudrate_));
    cfsetispeed(&tty, to_speed(baudrate_));

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;   // 8N1
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;    // 0.5s read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("tcsetattr failed");
    }

    // 启动读取线程
    read_thread_ = std::thread([this]() {
      ok_ = true;
      RCLCPP_INFO(logger_, "Serial read thread started on %s", device_.c_str());
      while (!quit_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        try {
          read_once();
        } catch (const std::exception & e) {
          RCLCPP_WARN(logger_, "Serial read error: %s", e.what());
          ok_ = false;
          break;
        }
      }
    });

    RCLCPP_INFO(logger_, "Serial opened: %s @ %d bps", device_.c_str(), baudrate_);
  }

  void try_open()
  {
    try {
      open_port();
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "Serial open failed: %s (will retry)", e.what());
    }
  }

  // -----------------------------------------------------------------------
  // 单次读取（select + read）
  // -----------------------------------------------------------------------
  void read_once()
  {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);

    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 10000;  // 10ms

    int ret = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (ret > 0 && FD_ISSET(fd_, &rfds)) {
      ssize_t n = ::read(fd_, read_buf_, sizeof(read_buf_));
      if (n > 0) {
        rx_callback_(read_buf_, static_cast<size_t>(n));
      } else if (n < 0) {
        throw std::runtime_error("read() error");
      }
    } else if (ret < 0) {
      throw std::runtime_error("select() error");
    }
  }

  void close_fd()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    ok_ = false;
  }
};

}  // namespace rm_hw_bridge

#endif  // RM_HW_BRIDGE__SERIAL_PORT_HPP_