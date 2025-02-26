#include <socket_transfer/client_udp.hpp>
#include "TFMessage.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<ClientUDP<TFMessage>>();
    rclcpp::spin(client);
}