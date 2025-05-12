#include <socket_transfer/service/server.hpp>
#include "OpenLLMRequest.hpp"


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<SocketTransfer::ServerService<OpenLLMRequest>>();
    client->Run();
}