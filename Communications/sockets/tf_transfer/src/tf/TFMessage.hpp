#pragma once
#include "tf2_msgs/msg/tf_message.hpp"
#include <MinimalSocket/core/Definitions.h>
#include <socket_transfer/serialization.hpp>

using geometry_msgs::msg::TransformStamped;
using tf2_msgs::msg::TFMessage;

template <>
struct SocketTransfer::Serializer<TFMessage>
{
    static MinimalSocket::BufferView Serialize(const TFMessage& msg, MinimalSocket::BufferView bufferView)
    {
        BufferWriter writer(bufferView.buffer, bufferView.buffer_size);

        size_t count = msg.transforms.size();
        writer.Write(&count);

        for (const TransformStamped& tfStmp : msg.transforms)
        {
            // Header
            SerializationUtils::SerializeHeader(writer, tfStmp.header);

            // child frame id
            SerializationUtils::SerializeString(writer, tfStmp.child_frame_id);

            // Transform
            writer.Write(&tfStmp.transform);
        }

        return writer.getUsedBufferView();
    }

    static void Deserialize(TFMessage& msg, MinimalSocket::BufferView bufferView)
    {
        BufferReader reader(bufferView.buffer, bufferView.buffer_size);

        size_t count = msg.transforms.size();
        reader.Read(&count);
        msg.transforms.resize(count);

        for (TransformStamped& tfStmp : msg.transforms)
        {
            // Header
            SerializationUtils::DeserializeHeader(reader, tfStmp.header);

            // child frame id
            SerializationUtils::DeserializeString(reader, tfStmp.child_frame_id);

            // Transform
            reader.Read(&tfStmp.transform);
        }
    }
};