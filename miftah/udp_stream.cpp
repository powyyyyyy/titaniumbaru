

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <arpa/inet.h>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>

#define STM32_ADDR "192.168.1.123"
#define STM32_PORT 1555
#define PC_ADDR    "192.168.1.124"
#define PC_PORT    1556
#define BUFF_SIZE  64

using namespace std::chrono_literals;

class UdpNode : public rclcpp::Node
{
public:
    UdpNode() : Node("udp_stream")
    {
        initSocket();

        // Lidar subscriber  (/obstacle_data from lidar node)
        sub_lidar_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/lidar_data", 10,
            std::bind(&UdpNode::lidarCallback, this, std::placeholders::_1));

        // YOLO subscriber  (/obstacle_data from test_cam.py)
        sub_yolo_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/obstacle_data", 10,
            std::bind(&UdpNode::yoloCallback, this, std::placeholders::_1));

            // write loop
        timer_write_ = this->create_wall_timer(
            1ms, std::bind(&UdpNode::udpWrite, this));

            // read loop
        timer_read_ = this->create_wall_timer(
            1ms, std::bind(&UdpNode::udpRead, this));

        RCLCPP_INFO(this->get_logger(), "UDP STREAM READY");
    }

    ~UdpNode() { close(sockfd_); }

private:
    int sockfd_;
    char rx_buffer_[BUFF_SIZE]{};
    char tx_buffer_[BUFF_SIZE]{};

    struct sockaddr_in pc_addr_;
    struct sockaddr_in stm32_addr_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_lidar_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_yolo_;
    rclcpp::TimerBase::SharedPtr timer_write_;
    rclcpp::TimerBase::SharedPtr timer_read_;

    // ── lidar state ──────────────────────────────────────────────────────────
    float front_ = 0.0f;
    float left_  = 0.0f;
    float right_ = 0.0f;

    // ── YOLO state ───────────────────────────────────────────────────────────
    float dx_   = 0.0f;
    float dy_   = 0.0f;
    float conf_ = 0.0f;
    float cls_id_ = 0.0f;

    void initSocket()
    {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);

        pc_addr_.sin_family      = AF_INET;
        pc_addr_.sin_addr.s_addr = inet_addr(PC_ADDR);
        pc_addr_.sin_port        = htons(PC_PORT);

        if (bind(sockfd_, (struct sockaddr *)&pc_addr_, sizeof(pc_addr_)) < 0)
            RCLCPP_ERROR(this->get_logger(), "Bind failed");

        stm32_addr_.sin_family      = AF_INET;
        stm32_addr_.sin_addr.s_addr = inet_addr(STM32_ADDR);
        stm32_addr_.sin_port        = htons(STM32_PORT);
    }

    void lidarCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 3) return;
        front_ = msg->data[0];
        left_  = msg->data[1];
        right_ = msg->data[2];
    }

    void yoloCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // [dx, dy, conf, dist_px, cls_id]
        if (msg->data.size() < 4) return;
        dx_   = msg->data[0];
        dy_   = msg->data[1];
        conf_ = msg->data[2];
        cls_id_ = msg->data[3];
    }

    void udpWrite()
    {
        memset(tx_buffer_, 0, BUFF_SIZE);

        // Header
        tx_buffer_[0] = 'A';
        tx_buffer_[1] = 'B';
        tx_buffer_[2] = 'C';

        // Lidar (bytes 3-14)
        memcpy(tx_buffer_ + 3,  &front_, 4);
        memcpy(tx_buffer_ + 7,  &left_,  4);
        memcpy(tx_buffer_ + 11, &right_, 4);

        // YOLO cartesian offsets (bytes 15-26)
        memcpy(tx_buffer_ + 15, &dx_,   4);
        memcpy(tx_buffer_ + 19, &dy_,   4);
        memcpy(tx_buffer_ + 23, &conf_, 4);
        memcpy(tx_buffer_ + 27, &cls_id_, 4);

        sendto(sockfd_, tx_buffer_, 31, 0,
               (struct sockaddr *)&stm32_addr_,
               sizeof(stm32_addr_));
    }

    void udpRead()
    {
        socklen_t len   = sizeof(stm32_addr_);
        int       bytes = recvfrom(sockfd_, rx_buffer_, BUFF_SIZE, MSG_DONTWAIT,
                                   (struct sockaddr *)&stm32_addr_, &len);
        if (bytes <= 0) return;

        if (rx_buffer_[0] != 'A' ||
            rx_buffer_[1] != 'B' ||
            rx_buffer_[2] != 'C')
        {
            return;
        }

        int16_t status = 0;
        memcpy(&status, rx_buffer_ + 3, 2);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpNode>());
    rclcpp::shutdown();
    return 0;
}
