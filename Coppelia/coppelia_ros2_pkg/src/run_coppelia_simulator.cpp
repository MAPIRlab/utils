#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>
#include <string>


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

      this->declare_parameter<bool>("coppelia_verbose", false);
      this->get_parameter("coppelia_verbose", coppelia_verbose);

      this->declare_parameter<bool>("autoplay", true);
      this->get_parameter("autoplay", autoplay);   

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
        std::vector<const char*> arguments;
        arguments.push_back("coppeliaSim"); //process name, doesn't really do anything
        arguments.push_back(coppelia_scene.c_str());

        if (coppelia_headless)
            arguments.push_back("-h");
        if(autoplay)
            arguments.push_back("-s");
        if(coppelia_verbose)
            arguments.push_back("-vinfos");
        else
            arguments.push_back("-vwarnings");


        arguments.push_back("&");
        arguments.push_back(nullptr); //signal end of argument list

        execv( (coppelia_dir+"/coppeliaSim.sh").c_str(), ((char**) arguments.data()) );
    }

    ~CoppeliaSim()
    {
    }
    
  private:
    std::string coppelia_dir, coppelia_scene;
    bool coppelia_headless;
    bool coppelia_verbose;
    bool autoplay;
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




