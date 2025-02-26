#pragma once
#include "MinimalSocket/core/Definitions.h"
#include "socket_transfer/serialization.hpp"
#include <sensor_msgs/msg/image.hpp>

using sensor_msgs::msg::Image;

template <>
inline MinimalSocket::BufferView Serialize<Image>(const Image& msg, MinimalSocket::BufferView bufferView)
{
    BufferWriter writer(bufferView.buffer, bufferView.buffer_size);

    // Header
    writer.Write(&msg.header.stamp.sec);
    writer.Write(&msg.header.stamp.nanosec);

    uint16_t frameIDSize = msg.header.frame_id.length();
    writer.Write(&frameIDSize);
    writer.Write(msg.header.frame_id.data(), frameIDSize);
    
    // metadata
    writer.Write(&msg.height);
    writer.Write(&msg.width);

    size_t encoding_length = msg.encoding.length();
    writer.Write(&encoding_length);
    writer.Write(msg.encoding.data(), encoding_length);

    writer.Write(&msg.is_bigendian);
    writer.Write(&msg.step);

    // data
    size_t dataSize = msg.data.size();
    writer.Write(&dataSize);

    writer.Write(msg.data.data(), dataSize);

    bufferView.buffer_size = writer.currentOffset();

    return bufferView;
}

template <>
inline void Deserialize<Image>(Image& msg, MinimalSocket::BufferView bufferView)
{
    BufferReader reader(bufferView.buffer, bufferView.buffer_size);

    //  Header
    reader.Read(&msg.header.stamp.sec);
    reader.Read(&msg.header.stamp.nanosec);

    uint16_t frameIDSize;
    reader.Read(&frameIDSize);
    msg.header.frame_id.resize(frameIDSize);
    reader.Read(msg.header.frame_id.data(), frameIDSize);

    // metadata
    reader.Read(&msg.height);
    reader.Read(&msg.width);

    size_t encoding_length;
    reader.Read(&encoding_length);
    msg.encoding.resize(encoding_length);
    reader.Read(msg.encoding.data(), encoding_length);

    reader.Read(&msg.is_bigendian);
    reader.Read(&msg.step);

    // data
    size_t dataSize;
    reader.Read(&dataSize);
    msg.data.resize(dataSize);

    reader.Read(msg.data.data(), dataSize);
}