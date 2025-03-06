#pragma once
#include "socket_transfer/serialization.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

using sensor_msgs::msg::LaserScan;

namespace SocketTransfer
{
    template <>
    struct Serializer<LaserScan>
    {
        static MinimalSocket::BufferView Serialize(const LaserScan& msg, MinimalSocket::BufferView bufferView)
        {
            BufferWriter writer(bufferView.buffer, bufferView.buffer_size);

            SerializationUtils::SerializeHeader(writer, msg.header);
            writer.Write(&msg.angle_min);
            writer.Write(&msg.angle_max);
            writer.Write(&msg.angle_increment);
            writer.Write(&msg.time_increment);
            writer.Write(&msg.scan_time);
            writer.Write(&msg.range_min);
            writer.Write(&msg.range_max);

            SerializationUtils::SerializeVector(writer, msg.ranges);
            SerializationUtils::SerializeVector(writer, msg.intensities);

            return writer.getUsedBufferView();
        }

        static void Deserialize(LaserScan& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView.buffer, bufferView.buffer_size);

            SerializationUtils::DeserializeHeader(reader, msg.header);
            
            reader.Read(&msg.angle_min);
            reader.Read(&msg.angle_max);
            reader.Read(&msg.angle_increment);
            reader.Read(&msg.time_increment);
            reader.Read(&msg.scan_time);
            reader.Read(&msg.range_min);
            reader.Read(&msg.range_max);
            SerializationUtils::DeserializeVector(reader, msg.ranges);
            SerializationUtils::DeserializeVector(reader, msg.intensities);
        }
    };
} // namespace SocketTransfer