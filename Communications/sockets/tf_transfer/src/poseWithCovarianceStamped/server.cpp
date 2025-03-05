#include <socket_transfer/topic/server.hpp>
#include "PoseWithCovarianceStamped.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    SocketTransfer::ServerTopic<PoseWithCovarianceStamped> server;
    server.Run();
}