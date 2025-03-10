#pragma once
#include <geometry_msgs/msg/twist.hpp>
#include <socket_transfer/serialization.hpp>

using geometry_msgs::msg::Twist;

template <>
struct SocketTransfer::Serializer<Twist>
{
    static MinimalSocket::BufferView Serialize(const Twist& msg, MinimalSocket::BufferView bufferView)
    {
        BufferWriter writer(bufferView);
        writer.Write(&msg);
        return writer.getUsedBufferView();
    }

    static void Deserialize(Twist& msg, MinimalSocket::BufferView bufferView)
    {
        BufferReader reader(bufferView);
        reader.Read(&msg);
    }
};