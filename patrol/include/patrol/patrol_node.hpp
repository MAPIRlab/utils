#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "patrol/action/patrol_times.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PatrolTimesServer : public rclcpp::Node{
    public:
        // for easy of use
        using PatrolTimes = patrol::action::PatrolTimes;
        using GoalHandlePatrolTimes = rclcpp_action::ServerGoalHandle<PatrolTimes>;
        using NavToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNavToPose = rclcpp_action::ClientGoalHandle<NavToPose>;

        PatrolTimesServer();
        ~PatrolTimesServer();
    private:
        // Patrol Action Server
        rclcpp_action::Server<PatrolTimes>::SharedPtr action_server_;

        // Declare goal, cancel, accept 
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const PatrolTimes::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandlePatrolTimes> goal_handle);
  
        void handle_accepted(const std::shared_ptr<GoalHandlePatrolTimes> goal_handle);
        void execute(const std::shared_ptr<GoalHandlePatrolTimes> goal_handle);


        // Nav2 action client
        rclcpp_action::Client<NavToPose>::SharedPtr nav_client_;
        void goal_response_callback(const GoalHandleNavToPose::SharedPtr & goal_handle);
        void feedback_callback(
                GoalHandleNavToPose::SharedPtr,
                const std::shared_ptr<const NavToPose::Feedback> feedback);
        void result_callback(const GoalHandleNavToPose::WrappedResult & result);

        // Patrol list of poses
        std::vector<geometry_msgs::msg::PoseStamped> pose_list_;
        bool nav_active_;
};