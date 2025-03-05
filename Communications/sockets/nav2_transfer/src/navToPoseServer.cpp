#include "NavToPose.hpp"
#include <socket_transfer/action/server.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    SocketTransfer::ServerAction<NavToPose> server;
    server.Run();
}