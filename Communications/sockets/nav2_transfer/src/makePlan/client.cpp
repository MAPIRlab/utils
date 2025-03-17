#include "ComputePathToPose.hpp"
#include <socket_transfer/action/client.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SocketTransfer::ClientAction<ComputePathToPose>>();
    node->Run();
}