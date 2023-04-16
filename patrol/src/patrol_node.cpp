#include <patrol/patrol_node.hpp>
#include <sstream>

using namespace std::placeholders;


PatrolTimesServer::PatrolTimesServer() : Node("patrol_times_action_server")
{
    RCLCPP_INFO(this->get_logger(), "Initializing PatrolTimes server...");
    using namespace std::placeholders;

    // init PatrolTimes action_server and its methods
    this->action_server_ = rclcpp_action::create_server<PatrolTimes>(
      this,
      "patrol_times",
      std::bind(&PatrolTimesServer::handle_goal, this, _1, _2),
      std::bind(&PatrolTimesServer::handle_cancel, this, _1),
      std::bind(&PatrolTimesServer::handle_accepted, this, _1)
    );

    // Create a nav2 NavigateToPose action client
    this->nav_client_ = rclcpp_action::create_client<NavToPose>(
      this,
      "navigate_to_pose"
    );

    // Init poses of the patrol from parameters
    RCLCPP_INFO(this->get_logger(), "Loading Poses of the Patrol...");
    int i = 1;
    std::vector<double> default_pose = {NAN, NAN, NAN, NAN, NAN, NAN, NAN};

    try
    {
      std::stringstream ss;
      int i = 1;
      while(true)
      {
        // pose_i
        ss.str(std::string()); // Clear the string stream
        ss << "pose_" << i;
        
        // declare and read parameter (or default nan)
        std::vector<double> p = this->declare_parameter<std::vector<double>>(ss.str(), default_pose);
        
        // check if pose_i has been set
        if (std::isnan(p[0])){
          // pose_i not found in paramters..
          break;
        }
        else{
          // show pose
          std::cout << ss.str() << " = [";
          for (double j: p)
            std::cout << j << ' ';
          std::cout << "]" << std::endl;

          // set as pose
          auto wp = geometry_msgs::msg::PoseStamped();
          wp.header.frame_id = "map";
          wp.header.stamp = this->now();

          if (p.size() == 7){
            // full pose provided
            wp.pose.position.x = p[0];
            wp.pose.position.y = p[1];
            wp.pose.position.z = p[2];;
            wp.pose.orientation.x = p[3];
            wp.pose.orientation.y = p[4];
            wp.pose.orientation.z = p[5];
            wp.pose.orientation.w = p[6];
          }else if (p.size() == 3)
          {
            // simple Position provided
            wp.pose.position.x = p[0];
            wp.pose.position.y = p[1];
            wp.pose.position.z = p[2];;
            wp.pose.orientation.x = 0;
            wp.pose.orientation.y = 0;
            wp.pose.orientation.z = 0;
            wp.pose.orientation.w = 1;
          }
          else
          {
            RCLCPP_INFO(this->get_logger(), "%s is not a pose(7) neither a position(3). Ignoring.", ss.str());
          }
          
          // add pose to list          
          pose_list_.push_back(wp);

          // another pose?
          i++;
        }
      }

      RCLCPP_INFO(this->get_logger(), "PatrolTimes server ready for operation. %u poses loaded. Waiting requests...",i-1);
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
}

PatrolTimesServer::~PatrolTimesServer(){}


// New Patrol request
rclcpp_action::GoalResponse PatrolTimesServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PatrolTimes::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with times= %d", goal->times);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Cancel Patrol request
rclcpp_action::CancelResponse PatrolTimesServer::handle_cancel(
    const std::shared_ptr<GoalHandlePatrolTimes> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

// Execute Patrol Request!
void PatrolTimesServer::handle_accepted(const std::shared_ptr<GoalHandlePatrolTimes> goal_handle)
{
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&PatrolTimesServer::execute, this, _1), goal_handle}.detach();
}

void PatrolTimesServer::execute(const std::shared_ptr<GoalHandlePatrolTimes> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal...");
    
    // Data to handle
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<PatrolTimes::Feedback>();
    auto result = std::make_shared<PatrolTimes::Result>();

    // Avoid blocking! Loop every 1s
    rclcpp::Rate loop_rate(1);
    auto & times_completed = feedback->times_completed;
    times_completed = 0;
    
    // Check NavigateToPose is ready!
    if (!this->nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose server not available after waiting. Canceling Patrol");
        this->nav_active_ = false;
        result->ack = false;
        result->msg = "NavigateRoPose not available after waiting";
        goal_handle->canceled(result);
        return;
    }

    // Execute the patroll routine "times" times
    for (int i = 1; (i <= goal->times) && rclcpp::ok(); ++i) 
    {
        RCLCPP_INFO(this->get_logger(), "Patrol number: %i",i);

        // Do a full patrol (all poses in pose_list_)
        for (auto p : pose_list_)
        {
            // Command the robot to the next pose
            auto goal_msg = NavToPose::Goal();
            goal_msg.pose = p; //PoseStamped

            // Declare callbacks
            auto goal_options = rclcpp_action::Client<NavToPose>::SendGoalOptions();
            goal_options.goal_response_callback =
            std::bind(&PatrolTimesServer::goal_response_callback, this, _1);
            goal_options.feedback_callback =
            std::bind(&PatrolTimesServer::feedback_callback, this, _1, _2);
            goal_options.result_callback =
            std::bind(&PatrolTimesServer::result_callback, this, _1);

            // Call Nav2 NavigateToPose
            this->nav_active_ = true;
            RCLCPP_INFO(this->get_logger(), "Navigating to pose [x,y]=[%.2f, %.2f]",p.pose.position.x, p.pose.position.y);
            auto goal_handle_future = this->nav_client_->async_send_goal(goal_msg, goal_options);
            
            // Wait for navigation to complete
            while (this->nav_active_) 
            {
                // IMPORTANT: Check if there is a cancel request for the Patrol action server!
                if (goal_handle->is_canceling()) 
                {
                    this->nav_active_ = false;
                    result->ack = false;
                    result->msg = "Unable to complete patrol. Canceled by request";
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Patrol canceled");

                    // request cancel to navToPose
                    this->nav_client_->async_cancel_all_goals();
                    return;
                }

                // Publish Patrol feedback
                times_completed = i-1;
                goal_handle->publish_feedback(feedback);

                // Check every 1s
                loop_rate.sleep();
            }
        }
    }

    // Patrol is done! inform the client!
    if (rclcpp::ok()) {
      result->ack = true;
      result->msg = "Patrol completed with success";
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}


// NAVTOPOSE CLIENT CALLBACKS
void PatrolTimesServer::goal_response_callback(const GoalHandleNavToPose::SharedPtr & goal_handle)
{
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "NavToPose Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "NavToPose Goal accepted by server, waiting for result");
    }
}

void PatrolTimesServer::feedback_callback(
        GoalHandleNavToPose::SharedPtr,
        const std::shared_ptr<const NavToPose::Feedback> feedback)
{
    // NavToPose feedback is the reaming distance to goal.pose (not really very interesting)
}

void PatrolTimesServer::result_callback(const GoalHandleNavToPose::WrappedResult & result)
{
    // NavToPose is completed! check result
    this->nav_active_ = false;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "NavToPose Goal was completed");
        return;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "NavToPose Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "NavToPose Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "NavToPose Unknown result code");
        return;
    }
}


// MAIN
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Create a server
    auto p = std::make_shared<PatrolTimesServer>();
    rclcpp::spin(p);

    // bye bye
    rclcpp::shutdown();
    return 0;
}