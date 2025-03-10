#pragma once
#include "MinimalSocket/core/Definitions.h"
#include "socket_transfer/serialization.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>

using sensor_msgs::msg::CompressedImage;

namespace SocketTransfer
{
    template <>
    struct Serializer<CompressedImage>
    {
        static MinimalSocket::BufferView Serialize(const CompressedImage& msg, MinimalSocket::BufferView bufferView)
        {
            BufferWriter writer(bufferView.buffer, bufferView.buffer_size);

            SerializationUtils::SerializeHeader(writer, msg.header);
            SerializationUtils::SerializeString(writer, msg.format);
            SerializationUtils::SerializeVector(writer, msg.data);

            return writer.getUsedBufferView();
        }

        static void Deserialize(CompressedImage& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView.buffer, bufferView.buffer_size);

            SerializationUtils::DeserializeHeader(reader, msg.header);
            SerializationUtils::DeserializeString(reader, msg.format);
            SerializationUtils::DeserializeVector(reader, msg.data);
        }
    };
} // namespace SocketTransfer