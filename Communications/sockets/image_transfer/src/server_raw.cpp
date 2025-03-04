#include "Image.hpp"
#include <socket_transfer/topic/server.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    SocketTransfer::ServerTopic<Image> server;
    server.Run();
}