#include "NavToPose.hpp"
#include <socket_transfer/action/client.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    SocketTransfer::ClientAction<NavToPose> client;
    client.Run();
}