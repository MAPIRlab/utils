#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <iomanip>
#include <string>
#include <sstream>

class CwebInterface: public rclcpp::Node
{
    public:
        CwebInterface(): Node("ChromeHRI")
        {
            //Read parameters and set default values
            this->declare_parameter<std::string>("script_id", "");
            this->get_parameter("script_id", script_id);
        }

        void launchScript()
        {
            // launch the chrome extension from terminal (no need to manually open Chrome)
            // The Chrome extension implements a MQTT client, reproducing all incoming msg on a specific topic
            // see extension configuration parameters

            // may throw ament_index_cpp::PackageNotFoundError exception
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("webHRI");

            std::ostringstream s;
            //s << "/opt/google/chrome/google-chrome --profile-directory=Default --app-id=" << script_id << " &";
            s << "/opt/google/chrome/google-chrome " << package_share_directory.c_str() << "/web/web_hri.html &";

            printf(s.str().c_str());

            system( s.str().c_str() );

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[chromeHRI] Opening Chrome Extension with command: %s", s.str().c_str() );
        }

    private:
        std::string script_id;
};

// Simpe executor to automatically load the Chrome Extension on ROS2 startup
int main(int argc, char** argv)
{
    // init
    rclcpp::init(argc, argv);

    // create object
    std::shared_ptr<CwebInterface> hri = std::make_shared<CwebInterface>();
    
    // Launchthe Chrome Extension
    hri->launchScript();
    rclcpp::spin(hri);

    rclcpp::shutdown();
    return 0;
}