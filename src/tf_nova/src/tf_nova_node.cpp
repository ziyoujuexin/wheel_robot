#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <math.h>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

using namespace std::chrono_literals;

class TfNovaDriver : public rclcpp::Node
{
public:
  TfNovaDriver() : Node("tf_nova_node"), running_(false)
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    frame_header_ = static_cast<uint8_t>(declare_parameter<int>("frame_header", 0x59));
    use_mm_format_ = declare_parameter<bool>("use_mm_format", false);
    frame_id_ = declare_parameter<std::string>("frame_id", "tf_nova_link");
    min_range_ = declare_parameter<double>("min_range", 0.1);
    max_range_ = declare_parameter<double>("max_range", 7.0);
    confidence_threshold_ = declare_parameter<int>("confidence_threshold", 0);
    topic_name_ = declare_parameter<std::string>("range_topic", "tf_nova/range");

    publisher_ = create_publisher<sensor_msgs::msg::Range>(topic_name_, rclcpp::SensorDataQoS());

    if (!openSerial())
    {
      RCLCPP_FATAL(get_logger(), "Failed to open TF-NOVA serial port %s", port_.c_str());
      rclcpp::shutdown();
      return;
    }

    running_ = true;
    reader_thread_ = std::thread(&TfNovaDriver::readLoop, this);
    RCLCPP_INFO(get_logger(), "TF-NOVA driver started on %s @ %d baud", port_.c_str(), baud_rate_);
  }

  ~TfNovaDriver() override
  {
    running_ = false;
    if (reader_thread_.joinable())
    {
      reader_thread_.join();
    }
    if (fd_ >= 0)
    {
      ::close(fd_);
    }
  }

private:
  bool openSerial()
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
      RCLCPP_ERROR(get_logger(), "Cannot open port %s", port_.c_str());
      return false;
    }

    termios tty {};
    if (tcgetattr(fd_, &tty) != 0)
    {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed for %s", port_.c_str());
      return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    speed_t speed = B115200;
    switch (baud_rate_)
    {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
#ifdef B460800
    case 460800: speed = B460800; break;
#endif
#ifdef B500000
    case 500000: speed = B500000; break;
#endif
#ifdef B921600
    case 921600: speed = B921600; break;
#endif
    default:
      RCLCPP_WARN(get_logger(), "Unsupported baud %d, falling back to 115200", baud_rate_);
      speed = B115200;
      break;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed for %s", port_.c_str());
      return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void readLoop()
  {
    std::vector<uint8_t> frame;
    frame.reserve(9);
    bool seen_first_header = false;

    while (rclcpp::ok() && running_)
    {
      uint8_t byte = 0;
      if (!readByte(byte))
      {
        std::this_thread::sleep_for(2ms);
        continue;
      }

      if (!seen_first_header)
      {
        if (byte == frame_header_)
        {
          seen_first_header = true;
          frame.clear();
          frame.push_back(byte);
        }
        continue;
      }

      // We already saw the first header byte
      if (frame.size() == 1)
      {
        if (byte == frame_header_)
        {
          frame.push_back(byte);
        }
        else
        {
          // Restart sync search
          seen_first_header = (byte == frame_header_);
          frame.clear();
          if (seen_first_header)
          {
            frame.push_back(byte);
          }
        }
        continue;
      }

      frame.push_back(byte);
      if (frame.size() < 9)
      {
        continue;
      }

      // We have 9 bytes; validate checksum
      uint8_t checksum = 0;
      for (size_t i = 0; i < 8; ++i)
      {
        checksum += frame[i];
      }

      if (checksum != frame[8])
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Checksum mismatch (calc %u, frame %u)", checksum, frame[8]);
        // Resync: keep last byte if it could be header
        seen_first_header = (frame.back() == frame_header_);
        frame.clear();
        if (seen_first_header)
        {
          frame.push_back(frame_header_);
        }
        continue;
      }

      publishFrame(frame);

      // Prepare for next frame
      seen_first_header = false;
      frame.clear();
    }
  }

  bool readByte(uint8_t &byte_out)
  {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100 ms timeout
    int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (ret > 0 && FD_ISSET(fd_, &read_fds))
    {
      ssize_t n = ::read(fd_, &byte_out, 1);
      if (n == 1)
      {
        return true;
      }
    }
    return false;
  }

  void publishFrame(const std::vector<uint8_t> &frame)
  {
    if (frame.size() != 9)
    {
      return;
    }

    uint16_t dist_raw = frame[2] | (static_cast<uint16_t>(frame[3]) << 8);
    uint16_t peak_raw = frame[4] | (static_cast<uint16_t>(frame[5]) << 8);
    int8_t temp_raw = static_cast<int8_t>(frame[6]);
    uint8_t confidence = frame[7];

    if (confidence < confidence_threshold_)
    {
      RCLCPP_DEBUG(get_logger(), "Skipping frame due to low confidence %u < %d", confidence,
                   confidence_threshold_);
      return;
    }

    double range_m = use_mm_format_ ? static_cast<double>(dist_raw) / 1000.0
                                    : static_cast<double>(dist_raw) / 100.0;

    if (range_m <= 0.0)
    {
      RCLCPP_DEBUG(get_logger(), "Received zero/negative range, skipping");
      return;
    }

    sensor_msgs::msg::Range msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view = 14.0 * M_PI / 180.0;  // horizontal FoV from manual
    msg.min_range = static_cast<float>(min_range_);
    msg.max_range = static_cast<float>(max_range_);
    msg.range = static_cast<float>(range_m);

    publisher_->publish(msg);

    RCLCPP_DEBUG(get_logger(),
                 "dist=%.3fm peak=%u temp=%dC conf=%u",
                 range_m, peak_raw, temp_raw, confidence);
  }

  std::string port_;
  int baud_rate_;
  uint8_t frame_header_;
  bool use_mm_format_;
  std::string frame_id_;
  double min_range_;
  double max_range_;
  int confidence_threshold_;
  std::string topic_name_;

  int fd_{-1};
  std::atomic<bool> running_;
  std::thread reader_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfNovaDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
