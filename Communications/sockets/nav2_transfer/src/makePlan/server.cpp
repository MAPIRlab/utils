#include "ComputePathToPose.hpp"
#include <socket_transfer/action/server.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SocketTransfer::ServerAction<ComputePathToPose>>();
    node->Run();
}