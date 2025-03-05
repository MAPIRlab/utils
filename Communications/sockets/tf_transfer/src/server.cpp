#include <socket_transfer/topic/server.hpp>
#include "TFMessage.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    SocketTransfer::ServerTopic<TFMessage> server;
    server.Run();
}