#pragma once
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <MinimalSocket/core/Definitions.h>
#include <socket_transfer/serialization.hpp>

using geometry_msgs::msg::PoseWithCovarianceStamped;

template <>
struct SocketTransfer::Serializer<PoseWithCovarianceStamped>
{
    static MinimalSocket::BufferView Serialize(const PoseWithCovarianceStamped& msg, MinimalSocket::BufferView bufferView)
    {
        BufferWriter writer(bufferView.buffer, bufferView.buffer_size);

        // Header
        SerializationUtils::SerializeHeader(writer, msg.header);

        // Pose
        writer.Write(&msg.pose);

        return writer.getUsedBufferView();
    }

    static void Deserialize(PoseWithCovarianceStamped& msg, MinimalSocket::BufferView bufferView)
    {
        BufferReader reader(bufferView.buffer, bufferView.buffer_size);

        // Header
        SerializationUtils::DeserializeHeader(reader, msg.header);

        // Pose
        reader.Read(&msg.pose);
    }
};