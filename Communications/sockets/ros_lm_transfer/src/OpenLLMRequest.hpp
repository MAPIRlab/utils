#pragma once
#include <ros_lm_interfaces/srv/open_llm_request.hpp>
#include <socket_transfer/serialization.hpp>

using ros_lm_interfaces::srv::OpenLLMRequest;

namespace SocketTransfer
{
    template <>
    struct Serializer<OpenLLMRequest::Request>
    {
        static MinimalSocket::BufferView Serialize(const OpenLLMRequest::Request& msg, MinimalSocket::BufferView bufferView)
        {
            BufferWriter writer(bufferView);
            writer.Write(&msg.action);
            SerializationUtils::SerializeString(writer, msg.model_id);
            SerializationUtils::SerializeString(writer, msg.prompt);
            writer.Write(&msg.max_length);
            writer.Write(&msg.temperature);
            writer.Write(&msg.top_k);
            writer.Write(&msg.top_p);
            SerializationUtils::SerializeVector(writer, msg.images);

            return writer.getUsedBufferView();
        }

        static void Deserialize(OpenLLMRequest::Request& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView);
            reader.Read(&msg.action);
            SerializationUtils::DeserializeString(reader, msg.model_id);
            SerializationUtils::DeserializeString(reader, msg.prompt);
            reader.Read(&msg.max_length);
            reader.Read(&msg.temperature);
            reader.Read(&msg.top_k);
            reader.Read(&msg.top_p);
            SerializationUtils::DeserializeVector(reader, msg.images);
        }
    };

    template <>
    struct Serializer<OpenLLMRequest::Response>
    {
        static MinimalSocket::BufferView Serialize(const OpenLLMRequest::Response& msg, MinimalSocket::BufferView bufferView)
        {
            BufferWriter writer(bufferView);
            writer.Write(&msg.status_code);
            SerializationUtils::SerializeString(writer, msg.status_message);
            SerializationUtils::SerializeString(writer, msg.generated_text);

            return writer.getUsedBufferView();
        }

        static void Deserialize(OpenLLMRequest::Response& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView);
            reader.Read(&msg.status_code);
            SerializationUtils::DeserializeString(reader, msg.status_message);
            SerializationUtils::DeserializeString(reader, msg.generated_text);
        }
    };
} // namespace SocketTransfer