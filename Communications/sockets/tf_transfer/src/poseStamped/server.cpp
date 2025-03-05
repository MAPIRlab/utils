#include <socket_transfer/topic/server.hpp>
#include "PoseStamped.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    SocketTransfer::ServerTopic<PoseStamped> server;
    server.Run();
}