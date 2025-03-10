#pragma once
#include "MinimalSocket/core/Definitions.h"
#include "socket_transfer/serialization.hpp"
#include <sensor_msgs/msg/camera_info.hpp>

using sensor_msgs::msg::CameraInfo;

namespace SocketTransfer
{
    template <>
    struct Serializer<CameraInfo>
    {
        static MinimalSocket::BufferView Serialize(const CameraInfo& msg, MinimalSocket::BufferView bufferView)
        {
            BufferWriter writer(bufferView.buffer, bufferView.buffer_size);

            SerializationUtils::SerializeHeader(writer, msg.header);

            writer.Write(&msg.height);
            writer.Write(&msg.width);

            SerializationUtils::SerializeString(writer, msg.distortion_model);
            SerializationUtils::SerializeVector(writer, msg.d);

            writer.Write(&msg.k);
            writer.Write(&msg.r);
            writer.Write(&msg.p);

            writer.Write(&msg.binning_x);
            writer.Write(&msg.binning_y);

            writer.Write(&msg.roi);

            return writer.getUsedBufferView();
        }

        static void Deserialize(CameraInfo& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView.buffer, bufferView.buffer_size);

            SerializationUtils::DeserializeHeader(reader, msg.header);

            reader.Read(&msg.height);
            reader.Read(&msg.width);

            SerializationUtils::DeserializeString(reader, msg.distortion_model);
            SerializationUtils::DeserializeVector(reader, msg.d);

            reader.Read(&msg.k);
            reader.Read(&msg.r);
            reader.Read(&msg.p);

            reader.Read(&msg.binning_x);
            reader.Read(&msg.binning_y);

            reader.Read(&msg.roi);
        }
    };
} // namespace SocketTransfer