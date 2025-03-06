#pragma once
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <MinimalSocket/core/Definitions.h>
#include <socket_transfer/serialization.hpp>

using geometry_msgs::msg::PoseStamped;

template <>
struct SocketTransfer::Serializer<PoseStamped>
{
    static MinimalSocket::BufferView Serialize(const PoseStamped& msg, MinimalSocket::BufferView bufferView)
    {
        BufferWriter writer(bufferView.buffer, bufferView.buffer_size);

        // Header
        SerializationUtils::SerializeHeader(writer, msg.header);

        // Pose
        writer.Write(&msg.pose);

        return writer.getUsedBufferView();
    }

    static void Deserialize(PoseStamped& msg, MinimalSocket::BufferView bufferView)
    {
        BufferReader reader(bufferView.buffer, bufferView.buffer_size);

        // Header
        SerializationUtils::DeserializeHeader(reader, msg.header);
        
        // Pose
        reader.Read(&msg.pose);
    }
};