#include "CameraInfo.hpp"
#include <socket_transfer/topic/server.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    SocketTransfer::ServerTopic<CameraInfo> server;
    server.Run();
}