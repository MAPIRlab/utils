#include "CompressedImage.hpp"
#include <socket_transfer/client_udp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<ClientUDP<CompressedImage>>();
    rclcpp::spin(client);
}