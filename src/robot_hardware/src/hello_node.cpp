#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

class SerialHelloNode : public rclcpp::Node
{
public:
  SerialHelloNode() : Node("serial_hello_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_ = declare_parameter<int>("baud", 115200);

    if (!openSerial()) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port %s", port_.c_str());
      rclcpp::shutdown();
      return;
    }

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SerialHelloNode::onTimer, this)
    );

    RCLCPP_INFO(get_logger(), "SerialHelloNode started (%s @ %d)",
                port_.c_str(), baud_);
  }

  ~SerialHelloNode()
  {
    if (fd_ >= 0) close(fd_);
  }

private:
  bool openSerial()
  {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) return false;

    struct termios tty {};
    if (tcgetattr(fd_, &tty) != 0) return false;

    cfmakeraw(&tty);

    speed_t speed = B115200;
    switch (baud_) {
      case 9600:    speed = B9600; break;
      case 19200:   speed = B19200; break;
      case 38400:   speed = B38400; break;
      case 57600:   speed = B57600; break;
      case 115200:  speed = B115200; break;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;  // 0.5s

    return (tcsetattr(fd_, TCSANOW, &tty) == 0);
  }

  void onTimer()
  {
    const std::string msg = "Hello World\n";
    ssize_t ret = write(fd_, msg.c_str(), msg.size());

    if (ret < 0) {
      RCLCPP_WARN(get_logger(), "Write failed");
    } else {
      RCLCPP_INFO(get_logger(), "Sent: %s", msg.c_str());
    }
  }

  std::string port_;
  int baud_;
  int fd_{-1};

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialHelloNode>());
  rclcpp::shutdown();
  return 0;
}
