#pragma once
#include "tf2_msgs/msg/tf_message.hpp"
#include <MinimalSocket/core/Definitions.h>
#include <socket_transfer/serialization.hpp>

using geometry_msgs::msg::TransformStamped;
using tf2_msgs::msg::TFMessage;

template <>
inline MinimalSocket::BufferView Serialize<TFMessage>(const TFMessage& msg, MinimalSocket::BufferView bufferView)
{
    BufferWriter writer(bufferView.buffer, bufferView.buffer_size);

    size_t count = msg.transforms.size();
    writer.Write(&count);

    for (const TransformStamped& tfStmp : msg.transforms)
    {
        // Header
        {
            writer.Write(&tfStmp.header.stamp.sec);
            writer.Write(&tfStmp.header.stamp.nanosec);

            uint16_t frameIDSize = tfStmp.header.frame_id.length();
            writer.Write(&frameIDSize);
            writer.Write(tfStmp.header.frame_id.data(), frameIDSize);
        }

        // child frame id
        {
            uint16_t childframeIDSize = tfStmp.child_frame_id.length();
            writer.Write(&childframeIDSize);
            writer.Write(tfStmp.child_frame_id.data(), childframeIDSize);
        }

        //Transform
        {
            writer.Write(&tfStmp.transform);
        }
    }

    bufferView.buffer_size = writer.currentOffset();

    return bufferView;
}

template <>
inline void Deserialize<TFMessage>(TFMessage& msg, MinimalSocket::BufferView bufferView)
{
    BufferReader reader(bufferView.buffer, bufferView.buffer_size);

    size_t count = msg.transforms.size();
    reader.Read(&count);
    msg.transforms.resize(count);

    for (TransformStamped& tfStmp : msg.transforms)
    {
        // Header
        {
            reader.Read(&tfStmp.header.stamp.sec);
            reader.Read(&tfStmp.header.stamp.nanosec);

            uint16_t frameIDSize = tfStmp.header.frame_id.length();
            reader.Read(&frameIDSize);
            tfStmp.header.frame_id.resize(frameIDSize);
            reader.Read(tfStmp.header.frame_id.data(), frameIDSize);
        }

        // child frame id
        {
            uint16_t childframeIDSize = tfStmp.child_frame_id.length();
            reader.Read(&childframeIDSize);
            tfStmp.child_frame_id.resize(childframeIDSize);
            reader.Read(tfStmp.child_frame_id.data(), childframeIDSize);
        }

        //Transform
        {
            reader.Read(&tfStmp.transform);
        }
    }
}