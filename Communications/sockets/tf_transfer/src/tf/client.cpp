#include <socket_transfer/topic/client.hpp>
#include "TFMessage.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    SocketTransfer::ClientTopic<TFMessage> client;
    client.Run();
}