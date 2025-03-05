#include "Map.hpp"
#include <socket_transfer/topic/server.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    SocketTransfer::ServerTopic<OccupancyGrid> server(rclcpp::QoS(1).transient_local());
    server.Run();
}