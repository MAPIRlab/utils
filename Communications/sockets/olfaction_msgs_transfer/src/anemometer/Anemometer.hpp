#pragma once
#include "socket_transfer/serialization.hpp"
#include <olfaction_msgs/msg/anemometer.hpp>

using olfaction_msgs::msg::Anemometer;

namespace SocketTransfer
{
    template <>
    struct Serializer<Anemometer>
    {
        static MinimalSocket::BufferView Serialize(const Anemometer& msg, MinimalSocket::BufferView bufferView)
        {
            BufferWriter writer(bufferView.buffer, bufferView.buffer_size);

            SerializationUtils::SerializeHeader(writer, msg.header);
            SerializationUtils::SerializeString(writer, msg.sensor_label);
            writer.Write(&msg.wind_speed);
            writer.Write(&msg.wind_direction);

            return writer.getUsedBufferView();
        }

        static void Deserialize(Anemometer& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView.buffer, bufferView.buffer_size);

            SerializationUtils::DeserializeHeader(reader, msg.header);
            SerializationUtils::DeserializeString(reader, msg.sensor_label);
            reader.Read(&msg.wind_speed);
            reader.Read(&msg.wind_direction);
        }
    };
} // namespace SocketTransfer