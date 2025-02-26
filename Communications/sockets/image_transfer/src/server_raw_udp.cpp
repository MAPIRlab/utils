#include "Image.hpp"
#include <socket_transfer/server_udp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto server = std::make_shared<ServerUDP<Image>>();
    server->Run();
}