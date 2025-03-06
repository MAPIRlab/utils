#pragma once
#include "MinimalSocket/core/Definitions.h"
#include "socket_transfer/serialization.hpp"
#include <sensor_msgs/msg/image.hpp>

using sensor_msgs::msg::Image;

namespace SocketTransfer
{
    template <>
    struct Serializer<Image>
    {
        static MinimalSocket::BufferView Serialize(const Image& msg, MinimalSocket::BufferView bufferView)
        {
            BufferWriter writer(bufferView.buffer, bufferView.buffer_size);

            // Header
            SerializationUtils::SerializeHeader(writer, msg.header);

            // metadata
            writer.Write(&msg.height);
            writer.Write(&msg.width);

            SerializationUtils::SerializeString(writer, msg.encoding);

            writer.Write(&msg.is_bigendian);
            writer.Write(&msg.step);

            // data
            SerializationUtils::SerializeVector(writer, msg.data);

            return writer.getUsedBufferView();
        }

        static void Deserialize(Image& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView.buffer, bufferView.buffer_size);

            //  Header
            SerializationUtils::DeserializeHeader(reader, msg.header);

            // metadata
            reader.Read(&msg.height);
            reader.Read(&msg.width);

            SerializationUtils::DeserializeString(reader, msg.encoding);

            reader.Read(&msg.is_bigendian);
            reader.Read(&msg.step);

            // data
            SerializationUtils::DeserializeVector(reader, msg.data);
        }
    };
} // namespace SocketTransfer
