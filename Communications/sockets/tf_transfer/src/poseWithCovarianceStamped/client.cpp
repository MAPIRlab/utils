#include <socket_transfer/topic/client.hpp>
#include "PoseWithCovarianceStamped.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    SocketTransfer::ClientTopic<PoseWithCovarianceStamped> client;
    client.Run();
}