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

            // Header
            writer.Write(&msg.header.stamp.sec);
            writer.Write(&msg.header.stamp.nanosec);

            uint16_t frameIDSize = msg.header.frame_id.length();
            writer.Write(&frameIDSize);
            writer.Write(msg.header.frame_id.data(), frameIDSize);

            // format
            uint16_t formatSize = msg.format.length();
            writer.Write(&formatSize);
            writer.Write(msg.format.data(), formatSize);

            // data
            size_t dataSize = msg.data.size();
            writer.Write(&dataSize);

            writer.Write(msg.data.data(), dataSize);

            bufferView.buffer_size = writer.currentOffset();

            return bufferView;
        }

        static void Deserialize(CompressedImage& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView.buffer, bufferView.buffer_size);

            //  Header
            reader.Read(&msg.header.stamp.sec);
            reader.Read(&msg.header.stamp.nanosec);

            uint16_t frameIDSize;
            reader.Read(&frameIDSize);
            msg.header.frame_id.resize(frameIDSize);
            reader.Read(msg.header.frame_id.data(), frameIDSize);

            // format
            uint16_t formatSize;
            reader.Read(&formatSize);
            msg.format.resize(formatSize);
            reader.Read(msg.format.data(), formatSize);

            // data
            size_t dataSize;
            reader.Read(&dataSize);
            msg.data.resize(dataSize);

            reader.Read(msg.data.data(), dataSize);
        }
    };
} // namespace SocketTransfer