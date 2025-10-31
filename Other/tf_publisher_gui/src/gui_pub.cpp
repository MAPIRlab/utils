#include <imgui_gl/imgui_gl.h>
#include <imgui/misc/cpp/imgui_stdlib.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

class GUIPub : public rclcpp::Node
{
public:
    GUIPub();
    void Run();

private:
    void RenderGUI();

private:
    std::string parent_frame;
    std::string child_frame;

    float freq;
    float x, y, z;
    float roll, pitch, yaw;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    bool render;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GUIPub>();
    node->Run();
}

inline GUIPub::GUIPub()
    : Node("guiPub")
{
    freq = declare_parameter<float>("freq", 10);
    x = declare_parameter<float>("x", 0);
    y = declare_parameter<float>("y", 0);
    z = declare_parameter<float>("z", 0);

    roll = declare_parameter<float>("roll", 0);
    pitch = declare_parameter<float>("pitch", 0);
    yaw = declare_parameter<float>("yaw", 0);

    parent_frame = declare_parameter<std::string>("parent_frame", "");
    child_frame = declare_parameter<std::string>("child_frame", "");
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    render = declare_parameter<bool>("renderGUI", true);
}

inline void GUIPub::Run()
{
    std::optional<std::jthread> renderThread;
    if (render)
        renderThread.emplace(std::bind(&GUIPub::RenderGUI, this));

    rclcpp::Rate rate(freq);
    while (rclcpp::ok())
    {
        if (parent_frame == "" || child_frame == "")
            continue;

        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = parent_frame;
        transform.header.stamp = now();
        transform.child_frame_id = child_frame;
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;

        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        transform.transform.rotation = tf2::toMsg(quat);

        tf_broadcaster->sendTransform(transform);
        rate.sleep();
    }
}

inline void GUIPub::RenderGUI()
{
    ImguiGL imgui;
    imgui.Setup(
        nullptr,
        "TransformPublisher",
        350,
        250,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiConfigFlags_NavEnableKeyboard);
    rclcpp::Rate rate(30);
    while (rclcpp::ok())
    {
        imgui.StartFrame();

        ImGuiIO& io = ImGui::GetIO();
        ImGui::SetNextWindowSize({io.DisplaySize.x, io.DisplaySize.y});
        ImGui::SetNextWindowPos(ImVec2(0, 0));

        ImGui::Begin("Parameters");
        ImGui::DragFloat("x", &x, 0.01f, -FLT_MAX, FLT_MAX, "%.2f");
        ImGui::DragFloat("y", &y, 0.01f, -FLT_MAX, FLT_MAX, "%.2f");
        ImGui::DragFloat("z", &z, 0.01f, -FLT_MAX, FLT_MAX, "%.2f");

        ImGui::DragFloat("roll", &roll, 0.01f, -FLT_MAX, FLT_MAX, "%.2f");
        ImGui::DragFloat("pitch", &pitch, 0.01f, -FLT_MAX, FLT_MAX, "%.2f");
        ImGui::DragFloat("yaw", &yaw, 0.01f, -FLT_MAX, FLT_MAX, "%.2f");

        ImGui::InputText("parent_frame", &parent_frame);
        ImGui::InputText("child_frame", &child_frame);

        if (ImGui::Button("Print Current config"))
        {
            fprintf(stderr, "{'x' :             %.2f},\n", x);
            fprintf(stderr, "{'y' :             %.2f},\n", y);
            fprintf(stderr, "{'z' :             %.2f},\n", z);
            fprintf(stderr, "{'roll' :          %.2f},\n", roll);
            fprintf(stderr, "{'pitch' :         %.2f},\n", pitch);
            fprintf(stderr, "{'yaw' :           %.2f},\n", yaw);
            fprintf(stderr, "{'parent_frame' :  '%s'},\n", parent_frame.c_str());
            fprintf(stderr, "{'child_frame' :   '%s'},\n", child_frame.c_str());
        }

        ImGui::End();

        imgui.Render();
        rate.sleep();
    }
}
