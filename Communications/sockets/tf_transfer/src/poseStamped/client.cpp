#include <socket_transfer/topic/client.hpp>
#include "PoseStamped.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    SocketTransfer::ClientTopic<PoseStamped> client;
    client.Run();
}