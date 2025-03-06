#pragma once
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <socket_transfer/serialization.hpp>

using NavToPose = nav2_msgs::action::NavigateToPose;

template <>
struct SocketTransfer::Serializer<NavToPose::Goal>
{
    static MinimalSocket::BufferView Serialize(const NavToPose::Goal& msg, MinimalSocket::BufferView bufferView)
    {
        BufferWriter writer(bufferView);

        // pose header
        SerializationUtils::SerializeHeader(writer, msg.pose.header);

        // pose
        writer.Write(&msg.pose.pose);

        SerializationUtils::SerializeString(writer, msg.behavior_tree);

        return writer.getUsedBufferView();
    }

    static void Deserialize(NavToPose::Goal& msg, MinimalSocket::BufferView bufferView)
    {
        BufferReader reader(bufferView);

        // pose header
        SerializationUtils::DeserializeHeader(reader, msg.pose.header);
        // pose
        reader.Read(&msg.pose.pose);

        // behavior tree
        SerializationUtils::DeserializeString(reader, msg.behavior_tree);
    }
};

template <>
struct SocketTransfer::Serializer<NavToPose::Result>
{
    static MinimalSocket::BufferView Serialize(const NavToPose::Result& msg, MinimalSocket::BufferView bufferView)
    {
        bufferView.buffer_size = 0;
        return bufferView;
    }

    static void Deserialize(NavToPose::Result& msg, MinimalSocket::BufferView bufferView)
    {}
};