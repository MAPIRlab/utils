#pragma once
#include "socket_transfer/serialization.hpp"
#include <olfaction_msgs/msg/gas_sensor.hpp>

using olfaction_msgs::msg::GasSensor;

namespace SocketTransfer
{
    template <>
    struct Serializer<GasSensor>
    {
        static MinimalSocket::BufferView Serialize(const GasSensor& msg, MinimalSocket::BufferView bufferView)
        {
            BufferWriter writer(bufferView.buffer, bufferView.buffer_size);

            SerializationUtils::SerializeHeader(writer, msg.header);
            writer.Write(&msg.technology);
            writer.Write(&msg.manufacturer);
            writer.Write(&msg.mpn);
            writer.Write(&msg.raw);
            writer.Write(&msg.raw_units);
            writer.Write(&msg.raw_air);
            writer.Write(&msg.calib_a);
            writer.Write(&msg.calib_b);

            return writer.getUsedBufferView();
        }

        static void Deserialize(GasSensor& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView.buffer, bufferView.buffer_size);

            SerializationUtils::DeserializeHeader(reader, msg.header);
            reader.Read(&msg.technology);
            reader.Read(&msg.manufacturer);
            reader.Read(&msg.mpn);
            reader.Read(&msg.raw);
            reader.Read(&msg.raw_units);
            reader.Read(&msg.raw_air);
            reader.Read(&msg.calib_a);
            reader.Read(&msg.calib_b);
        }
    };
} // namespace SocketTransfer