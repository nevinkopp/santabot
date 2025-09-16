#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>

class testdrive : public rclcpp::Node
{
public:
    testdrive() : Node("testdrive")
    {
        // Open serial port
        serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            throw std::runtime_error("Serial open failed");
        }
        configure_serial();

        // Put Create 2 into "safe mode"
        sendByte(128); // Start OI
        sendByte(131); // Safe mode

        // Subscribe to cmd_vel
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&Create2Driver::cmdVelCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Create 2 driver initialized");
    }

    ~Create2Driver()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    int serial_fd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    void configure_serial()
    {
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
            return;
        }

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
        tty.c_iflag &= ~IGNBRK;                     // disable break processing
        tty.c_lflag = 0;                            // no signaling chars, no echo
        tty.c_oflag = 0;                            // no remapping, no delays
        tty.c_cc[VMIN] = 0;                         // non-blocking read
        tty.c_cc[VTIME] = 5;                        // 0.5 sec timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // shut off xon/xoff ctrl
        tty.c_cflag |= (CLOCAL | CREAD);            // ignore modem controls
        tty.c_cflag &= ~(PARENB | PARODD);          // no parity
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
        }
    }

    void sendByte(uint8_t value)
    {
        write(serial_fd_, &value, 1);
    }

    void sendDriveCommand(int16_t velocity, int16_t radius)
    {
        uint8_t cmd[5];
        cmd[0] = 137; // Drive opcode
        cmd[1] = (velocity >> 8) & 0xFF;
        cmd[2] = velocity & 0xFF;
        cmd[3] = (radius >> 8) & 0xFF;
        cmd[4] = radius & 0xFF;
        write(serial_fd_, cmd, 5);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear = msg->linear.x;   // m/s
        double angular = msg->angular.z; // rad/s

        // Convert to mm/s (Create 2 expects mm/s)
        int16_t velocity = static_cast<int16_t>(linear * 1000.0);

        // Approximate radius (mm)
        if (angular == 0.0) {
            sendDriveCommand(velocity, 0x8000); // straight
        } else {
            int16_t radius = static_cast<int16_t>(velocity / angular);
            sendDriveCommand(velocity, radius);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Create2Driver>());
    rclcpp::shutdown();
    return 0;
}
