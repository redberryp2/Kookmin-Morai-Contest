#include <iostream>
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "local_pkg/PosPos.h"
#include <local_pkg/VescStateStamped.h>
#include <local_pkg/VescState.h>
#include <algorithm>
#include <fstream>
#include <memory>

using namespace std;


class Control : public ros::NodeHandle
{
    public:
    Control() : outfile("/home/autonav/km_ws/src/code/map/final_map3_obs.txt")
    {   
        this->gnss_sub_=this->subscribe("/PosCal",10,&Control::GNSSCallback, this);
        // this->speed_pub_ = this->advertise<std_msgs::Float64>("/commands/motor/speed",10);  
        // this->steer_pub_ = this->advertise<std_msgs::Float64>("/commands/servo/position",10); 
    }

    private:
    ros::Subscriber gnss_sub_;
    // ros::Publisher speed_pub_;
    // ros::Publisher steer_pub_;
    std_msgs::Float64 data_msg_;
    local_pkg::PosPos pos_msg_;
    ofstream outfile;

    
    
    void GNSSCallback(const local_pkg::PosPos::ConstPtr &msg)
    {   
        outfile << fixed;
        outfile.precision(15);
        outfile << msg -> PosX << "\t" << msg -> PosY << endl;
        
    }

};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"Path_Logger");
    auto control = make_shared<Control>();
    ros::spin();
    return 0;
}
