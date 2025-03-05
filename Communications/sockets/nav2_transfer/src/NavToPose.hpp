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
        {
            size_t frame_id_length = msg.pose.header.frame_id.length();
            writer.Write(&frame_id_length);
            writer.Write(msg.pose.header.frame_id.data(), frame_id_length);
            writer.Write(&msg.pose.header.stamp);
        }

        // pose
        {
            writer.Write(&msg.pose.pose);
        }

        {
            // behavior tree
            size_t behaviorTreeLength = msg.behavior_tree.length();
            writer.Write(&behaviorTreeLength);
            writer.Write(msg.behavior_tree.data(), behaviorTreeLength);
        }

        return writer.getUsedBufferView();
    }

    static void Deserialize(NavToPose::Goal& msg, MinimalSocket::BufferView bufferView)
    {
        BufferReader reader(bufferView);

        // pose header
        {
            size_t frame_id_length = msg.pose.header.frame_id.length();
            reader.Read(&frame_id_length);
            msg.pose.header.frame_id.resize(frame_id_length);
            reader.Read(msg.pose.header.frame_id.data(), frame_id_length);
            reader.Read(&msg.pose.header.stamp);
        }

        // pose
        {
            reader.Read(&msg.pose.pose);
        }

        // behavior tree
        {
            size_t behaviorTreeLength = msg.behavior_tree.length();
            reader.Read(&behaviorTreeLength);
            msg.behavior_tree.resize(behaviorTreeLength);
            reader.Read(msg.behavior_tree.data(), behaviorTreeLength);
        }
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