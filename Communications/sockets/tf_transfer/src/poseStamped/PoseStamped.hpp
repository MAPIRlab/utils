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
        {
            writer.Write(&msg.header.stamp.sec);
            writer.Write(&msg.header.stamp.nanosec);

            uint16_t frameIDSize = msg.header.frame_id.length();
            writer.Write(&frameIDSize);
            writer.Write(msg.header.frame_id.data(), frameIDSize);
        }

        // Pose
        {
            writer.Write(&msg.pose);
        }

        return writer.getUsedBufferView();
    }

    static void Deserialize(PoseStamped& msg, MinimalSocket::BufferView bufferView)
    {
        BufferReader reader(bufferView.buffer, bufferView.buffer_size);

        // Header
        {
            reader.Read(&msg.header.stamp.sec);
            reader.Read(&msg.header.stamp.nanosec);

            uint16_t frameIDSize = msg.header.frame_id.length();
            reader.Read(&frameIDSize);
            msg.header.frame_id.resize(frameIDSize);
            reader.Read(msg.header.frame_id.data(), frameIDSize);
        }

        // Pose
        {
            reader.Read(&msg.pose);
        }
    }
};