#pragma once
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <socket_transfer/serialization.hpp>

using nav_msgs::msg::OccupancyGrid;

template <>
struct SocketTransfer::Serializer<OccupancyGrid>
{
    static MinimalSocket::BufferView Serialize(const OccupancyGrid& msg, MinimalSocket::BufferView bufferView)
    {
        BufferWriter writer(bufferView);
        // header
        {
            size_t frame_id_length = msg.header.frame_id.length();
            writer.Write(&frame_id_length);
            writer.Write(msg.header.frame_id.data(), frame_id_length);
            writer.Write(&msg.header.stamp);
        }

        // metadata
        {
            writer.Write(&msg.info);
        }

        // data
        {
            size_t size = msg.data.size();
            writer.Write(&size);
            writer.Write(msg.data.data(), size);
        }

        return writer.getUsedBufferView();
    }

    static void Deserialize(OccupancyGrid& msg, MinimalSocket::BufferView bufferView)
    {

        BufferReader reader(bufferView);
        // header
        {
            size_t frame_id_length = msg.header.frame_id.length();
            reader.Read(&frame_id_length);
            msg.header.frame_id.resize(frame_id_length);
            reader.Read(msg.header.frame_id.data(), frame_id_length);
            reader.Read(&msg.header.stamp);
        }

        // metadata
        {
            reader.Read(&msg.info);
        }

        // data
        {
            size_t size = msg.data.size();
            reader.Read(&size);
            msg.data.resize(size);
            reader.Read(msg.data.data(), size);
        }
    }
};