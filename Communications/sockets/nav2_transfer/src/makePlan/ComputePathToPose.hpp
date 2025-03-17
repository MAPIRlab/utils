#pragma once

#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <socket_transfer/serialization.hpp>

using nav2_msgs::action::ComputePathToPose;

namespace SocketTransfer
{
    template <>
    struct Serializer<ComputePathToPose::Goal>
    {
        static MinimalSocket::BufferView Serialize(const ComputePathToPose::Goal& msg, MinimalSocket::BufferView bufferView)
        {
            BufferWriter writer(bufferView);
            SerializationUtils::SerializeHeader(writer, msg.goal.header);
            writer.Write(&msg.goal.pose);
            SerializationUtils::SerializeHeader(writer, msg.start.header);
            writer.Write(&msg.start.pose);

            SerializationUtils::SerializeString(writer, msg.planner_id);
            writer.Write(&msg.use_start);

            return writer.getUsedBufferView();
        }

        static void Deserialize(ComputePathToPose::Goal& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView);
            SerializationUtils::DeserializeHeader(reader, msg.goal.header);
            reader.Read(&msg.goal.pose);
            SerializationUtils::DeserializeHeader(reader, msg.start.header);
            reader.Read(&msg.start.pose);

            SerializationUtils::DeserializeString(reader, msg.planner_id);
            reader.Read(&msg.use_start);
        }
    };

    template <>
    struct Serializer<ComputePathToPose::Result>
    {
        static MinimalSocket::BufferView Serialize(const ComputePathToPose::Result& msg, MinimalSocket::BufferView bufferView)
        {
            BufferWriter writer(bufferView);

            writer.Write(&msg.planning_time);
            SerializationUtils::SerializeHeader(writer, msg.path.header);
            SerializationUtils::SerializeVector(writer, msg.path.poses);

            return writer.getUsedBufferView();
        }

        static void Deserialize(ComputePathToPose::Result& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView);

            reader.Read(&msg.planning_time);
            SerializationUtils::DeserializeHeader(reader, msg.path.header);
            SerializationUtils::DeserializeVector(reader, msg.path.poses);
        }
    };
} // namespace SocketTransfer