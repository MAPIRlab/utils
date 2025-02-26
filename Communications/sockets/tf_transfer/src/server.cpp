#include <socket_transfer/server_udp.hpp>
#include "TFMessage.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto server = std::make_shared<ServerUDP<TFMessage>>();
    server->Run();
}