#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>
#include <string>
#include <RemoteAPIClient.h>
#include <RemoteAPIObjects.h>


class CoppeliaSim: public rclcpp::Node
{
  public:
    CoppeliaSim()
    : Node("coppelia_simulator")
    {
      //Declare parameters and set default values
      this->declare_parameter<std::string>("coppelia_root_dir", "");
      this->get_parameter("coppelia_root_dir", coppelia_dir);

      this->declare_parameter<std::string>("coppelia_scene_path", "");
      this->get_parameter("coppelia_scene_path", coppelia_scene);
      
      this->declare_parameter<bool>("coppelia_headless", false);
      this->get_parameter("coppelia_headless", coppelia_headless);

#if USE_API
      this->declare_parameter<bool>("autoplay", true);
      this->get_parameter("autoplay", autoplay);      
#endif

      // Debug
      RCLCPP_INFO(this->get_logger(),"coppelia_root_dir: %s", coppelia_dir.c_str());
      RCLCPP_INFO(this->get_logger(),"coppelia_scene_path: %s", coppelia_scene.c_str());
      RCLCPP_INFO(this->get_logger(),"coppelia_headless: %u", coppelia_headless);

      // Declare other params (for the scene)
      this->declare_parameter<bool>("show_laser", false);
      this->declare_parameter<bool>("tf_gt", false);
      this->declare_parameter<bool>("tf_odom", true);
      this->declare_parameter<int>("num_beacons", 5);
      this->declare_parameter<float>("var_v",0.0);
      this->declare_parameter<float>("var_w",0.0);
    }
    
    void run()
    {
#if USE_API
        //we are calling fork, so there will be two processes: one that runs coppelia, and one that interacts with it through the remote api
        int this_pid = fork();
        if(this_pid!=0) //parent process
        {
#endif
            if (coppelia_headless){
                execl( (coppelia_dir+"/coppeliaSim.sh").c_str(), 
                "-s", coppelia_scene.c_str(), "-h", "&", 
                (char*)0 );
            }
            else{
                execl( (coppelia_dir+"/coppeliaSim.sh").c_str(), 
                "-s", coppelia_scene.c_str(), "&", 
                (char*)0 );
            }
#if USE_API
        }
        else //child process
        {
            client = std::make_unique<RemoteAPIClient>();
            sim = std::make_unique<RemoteAPIObject::sim>(client.get());
            if(autoplay)
                sim->startSimulation();
            else
                sim->stopSimulation();
        }
#endif
    }

    ~CoppeliaSim()
    {
    }
    
  private:
    std::string coppelia_dir, coppelia_scene;
    bool coppelia_headless;
#if USE_API
    bool autoplay;
    std::unique_ptr<RemoteAPIClient> client;
    std::unique_ptr<RemoteAPIObject::sim> sim;
#endif
};

      
// MAIN
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  {
    //create object
    std::shared_ptr<CoppeliaSim> myNode = std::make_shared<CoppeliaSim>();

    //launch simulator
    myNode->run();

    // Iterate (for param_server and related services)
    printf("[CoppeliaSimulator] Simulator launched. Now spinning!");
    rclcpp::spin(myNode);

    //destructor gets called
  }

  rclcpp::shutdown();
  return 0;
  
}




