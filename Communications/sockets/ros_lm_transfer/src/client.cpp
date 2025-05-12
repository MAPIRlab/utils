#include <socket_transfer/service/client.hpp>
#include "OpenLLMRequest.hpp"


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<SocketTransfer::ClientServer<OpenLLMRequest>>();
    client->Run();
}