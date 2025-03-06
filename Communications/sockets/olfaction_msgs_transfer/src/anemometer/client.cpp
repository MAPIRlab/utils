#include "Anemometer.hpp"
#include <socket_transfer/topic/client.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    SocketTransfer::ClientTopic<Anemometer> client;
    client.Run();
}