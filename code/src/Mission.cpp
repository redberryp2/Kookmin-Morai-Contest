#include <ros/ros.h>
#include <iostream>
#include <ros/package.h>
#include "std_msgs/Float64.h"
#include "local_pkg/PosPos.h"
#include "local_pkg/VescStateStamped.h"
#include "local_pkg/VescState.h"
#include "Lidar_pkg/pointCloud.h"
#include "Lidar_pkg/pointinfo.h"
#include "wego/parking.h"
#include "dh_pkg/cam_msg.h"
#include "morai_msgs/GetTrafficLightStatus.h"
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <fstream>
#include <string>
#include <memory>
#include <vector>
#include <cmath>

using namespace std;
string global_path = ros::package::getPath("code");
ifstream path(global_path+"/map/final_map3.txt");
ifstream path2(global_path+"/map/final_map3_obs.txt");

ros::Subscriber gnss_sub_;
ros::Subscriber lidar_sub_;
ros::Subscriber cam_sub_;
ros::Subscriber traffic_sub_;
ros::Subscriber parking_sub_;
ros::Publisher speed_pub_;
ros::Publisher steer_pub_;
std_msgs::Float64 speed_msg_;
std_msgs::Float64 steer_msg_;
local_pkg::PosPos pos_msg_;
morai_msgs::GetTrafficLightStatus traf_msg_;
wego::parking parking_msg_;

bool isparking = false;
bool realparking = false;
int parking_cnt = 0;

bool Mission_1_Static = false;
vector<vector<double>> Map_Data(6677,vector<double>(2,0));
vector<vector<double>> Obs_Map_Data(2833,vector<double>(2,0));
vector<double> My_Current_Point(2,0);
vector<double> Fixed_Current_Point(2,0);
vector<double> My_Current_WayPoint(2,0);
vector<double> My_Current_WayPoint2(2,0);

bool new_flag = false;
int size;
vector<vector<double>> Lidar_XYd, Lidar_Ymm,Lidar_ENU; 
bool Is_Dynamic_Obs = false;
bool Is_Dynamic_Obs_Flag = false;
int Is_Dynamic_Obs_Count = 0;
int Mission_1_Count = 0;
bool Missin_1_flag =false;
int slam_cnt =0;
bool slam_flag =false;
bool obs_flag = false;
int obs_cnt = 0;
bool tmp_flag =false;
int tmp_cnt = 0;
bool lottari_stop_flag = false;
bool lottari_stop_cnt = 0; 
bool traffic_stop_flag = false;

bool Static_obs =false;
int Static_obs_cnt = 0;

double borderline = 0.1;  
int local_min_idx = 0;

bool line1 = false;
bool line2 = false;

bool slip_mission2 = false;
bool slip_mission3 = false;
bool slip_traffic = false;
bool slip_lottari = false;


int slip_mission2_count = 0;
int slip_mission3_count = 0;
int slip_traffic_count = 0;
int slip_lottari_count = 0;
int count_Idx;

vector<double> local_Way_point(2,0);  
int lidar_cnt = 0;
double My_Current_Heading = 0;
int Min_Idx = 0;
int Min_Idx2 = 0;
double Delta;
double Delta2;
// float LD = 1.0;  // 1700
float LD = 1.2;  // 2000

void Flag_control()
{
    if(Min_Idx >= 3790 && Min_Idx <= 3795) 
    {
        lottari_stop_flag = true;
    }
    if((Min_Idx > 1700 && Min_Idx < 1710) || (Min_Idx2 > 1936 && Min_Idx2 < 1991))
    {
        line1 = true;
        line2 = false;
    }
    if((Min_Idx > 2800)|| (Min_Idx2 > 2800))
    {
        line1 = true;
        line2 = false;
    }
}
double dist(int i, vector<vector<double>> &Map_data, vector<double> &My_enu)
{
    return sqrt(pow(Map_data[i][0]-My_enu[0],2)+pow(Map_data[i][1]-My_enu[1],2));
}

void LIDARCallback(const Lidar_pkg::pointCloud::ConstPtr &msg)
{
    Lidar_XYd.clear();
    Lidar_Ymm.clear();
    size = msg -> size;
    if(obs_flag)
    {
        obs_cnt++;
    }
    if(obs_cnt % 15 ==0)
    {
        line1 = true;
        line2 = false;
    }
    if(size)  
    {
        if(1100 < Min_Idx && Min_Idx < 1796)
        {
            Mission_1_Static = true;
        }
        
        obs_flag =true;
        Static_obs = true;
        Lidar_XYd.resize(size,vector<double>(3,0));
        Lidar_Ymm.resize(size,vector<double>(2,0));
        Lidar_ENU.resize(size,vector<double>(2,0));
        for(int i = 0 ; i < size; i++)
        {
            Lidar_XYd[i][0] = msg ->points[i].x;
            Lidar_XYd[i][1] = msg ->points[i].y;
            Lidar_XYd[i][2] = msg ->points[i].dist;
            Lidar_Ymm[i][0] = msg ->points[i].y_Max;
            Lidar_Ymm[i][1] = msg ->points[i].y_Min;

            Lidar_ENU[i][0] = My_Current_Point[0] + cos((90-My_Current_Heading)*M_PI/180) * Lidar_XYd[i][0] - sin((90-My_Current_Heading)*M_PI/180)*Lidar_XYd[i][1]; 
            Lidar_ENU[i][1] = My_Current_Point[1] + sin((90-My_Current_Heading)*M_PI/180) * Lidar_XYd[i][0] + cos((90-My_Current_Heading)*M_PI/180)*Lidar_XYd[i][1];
        }
        if((Min_Idx > 1060 && Min_Idx < 1796))
        {
            if(Lidar_ENU[0][0]>9.78)
            {
                line2 = true;
                line1 = false;
            }
            else if(Lidar_ENU[0][0] <9.78)
            {
                line1 = true;
                line2 = false;
            }
        }
        else if((Min_Idx > 1700 && Min_Idx < 2900))
        {
        if(Lidar_ENU[0][1] <37.7551)  
        {
            line1 = true;
            line2 = false;
        }
        else if(Lidar_ENU[0][1] > 37.7551)
        {
            line2 = true;
            line1 = false;
        }
        }    
    }
    else
    {
        Static_obs = false;
    }
    if(Mission_1_Static)
    {
        if(Min_Idx > 1800 && Min_Idx < 2900)
        {
            line2 = false;
            line1 = true;;
        }
    }
    
}

void CAMCallback(const dh_pkg::cam_msg::ConstPtr &msg)
{
    Is_Dynamic_Obs = msg->is_dynamic_obs;
    // cout<<"hi"<<Mission_1_Static<<endl;

    if(Is_Dynamic_Obs == true)
    {   
        if(Min_Idx < 1700 || (Min_Idx > 1900 && Min_Idx < 2500))
        {
            Is_Dynamic_Obs_Flag = true;
        }
    }
    // cout<<"미션: "<<Misson<<" 정지선 여부:"<<Is_Stop_Flag<<" 동적 여부:"<<Is_Dynamic_Obs<<" 미션 시간:"<<Mission_TIme<<endl;
}
    
void GNSSCallback(const local_pkg::PosPos::ConstPtr &msg)
{   
    My_Current_Point[0] = msg->PosX;
    My_Current_Point[1] = msg->PosY;
    My_Current_Point[0] = My_Current_Point[0] + Fixed_Current_Point[0]; 
    My_Current_Point[1] = My_Current_Point[1] + Fixed_Current_Point[1]; 
    My_Current_Heading = msg->heading;
    My_Current_Heading = -My_Current_Heading ;

    if(slip_mission2 == true && (count_Idx + 50) < Min_Idx) {

        slip_mission2_count ++;
        slip_mission2 = false;
    }

    if(slip_mission3 == true && (count_Idx + 50) < Min_Idx) {
        
        slip_mission3_count ++;
        slip_mission3 = false;
    }

    if(slip_traffic == true && (count_Idx + 50) < Min_Idx) {

        slip_traffic_count ++;
        slip_traffic = false;
    }
    if(slip_lottari == true && (count_Idx + 100) < Min_Idx) {

    slip_lottari_count ++;
    slip_lottari = false;
    }

    // cout << "미션2 동적:" << slip_mission2_count << endl;
    // cout << "미션3 동적:" << slip_mission3_count << endl;
    // cout << "신호등:" << slip_traffic_count << endl;
    // cout << "로타리:" << slip_lottari_count << endl;
    My_Current_Point[0] = My_Current_Point[0] + (0.03 * slip_mission3_count) - (0.01 * slip_lottari_count);
    My_Current_Point[1] = My_Current_Point[1] - (0.03 * slip_mission2_count) + (0.04 * slip_traffic_count);
}

void TrafficCallback(const morai_msgs::GetTrafficLightStatus::ConstPtr &msg)
{
    string traffic_index = msg -> trafficLightIndex;
    int traffic_status = msg -> trafficLightStatus;
    if(traffic_index == "SN000005" && Min_Idx > 4456 && Min_Idx < 4480)
    {
        if(traffic_status == 33)
        {
            traffic_stop_flag =false;
        }
        else 
        {
            traffic_stop_flag =true;
        }
    }
}

void ParkingCallback(const wego::parking::ConstPtr &msg)
{
    isparking = msg -> parking_flag;
    if(isparking)
    {
        realparking =true;
        if(parking_cnt < 100)
        {
            parking_cnt++;
        }
    }

}
void Find_Delta_1()
{
    float Min_Dist = int(1e9);
    for (int i = Min_Idx; i < Min_Idx + 100; i++ )
    {
        if (Min_Dist > sqrt(pow(Map_Data[i][0] - My_Current_Point[0], 2) + pow(Map_Data[i][1] - My_Current_Point[1], 2)))
        {
            Min_Dist = sqrt(pow(Map_Data[i][0] - My_Current_Point[0], 2) + pow(Map_Data[i][1] - My_Current_Point[1], 2));
            Min_Idx = i;
            
        }
    }
    if(Min_Idx > 1780 && Min_Idx < 1880)  //처음 곡선
    {
        LD = 0.9;
    }
    if(Min_Idx > 1880 && Min_Idx < 2780 && Mission_1_Static == false)  //처음 곡선
    {
        LD = 1.1;
    }
    else if(Min_Idx > 2780 && Min_Idx < 2780)  //두번째 곡선
    {
        LD = 0.9;
    }
    else if(Min_Idx > 3250 && Min_Idx < 3350)  //세번째 곡선
    {
        LD = 0.9;
    }
    else if(Min_Idx > 4400 && Min_Idx < 4600)  //신호등
    {
        LD = 0.9;
    }
    else if(Min_Idx > 5600 && Min_Idx < 5650)  //네번쨰
    {
        LD = 0.9;
    }
    else if(Min_Idx > 6200 && Min_Idx < 6250)  //다섯번째
    {
        LD = 0.9;
    }
    else{
        LD = 1.05;
    }
    for (int i = Min_Idx; i < Map_Data.size(); i++)
    {
        if (LD < sqrt(pow(Map_Data[i][0] - My_Current_Point[0], 2) + pow(Map_Data[i][1] - My_Current_Point[1], 2)))
        {
            My_Current_WayPoint[0] = Map_Data[i][0];
            My_Current_WayPoint[1] = Map_Data[i][1];
            break;
        }
    }

    double alpha = atan2(My_Current_WayPoint[0] - My_Current_Point[0], My_Current_WayPoint[1] - My_Current_Point[1]);
    alpha = alpha - My_Current_Heading / 180.0 * M_PI;
    Delta = atan2(2 * sin(alpha) * 0.38, LD);
    Delta = Delta * 90 /(19.5 * M_PI);
    // cout<<Min_Idx<<endl;
}

void Find_Delta_2()
{
    float Min_Dist = int(1e9);
    for (int i = Min_Idx2; i < Min_Idx2 + 100; i++ )
    {
        if (Min_Dist > sqrt(pow(Obs_Map_Data[i][0] - My_Current_Point[0], 2) + pow(Obs_Map_Data[i][1] - My_Current_Point[1], 2)))
        {
            Min_Dist = sqrt(pow(Obs_Map_Data[i][0] - My_Current_Point[0], 2) + pow(Obs_Map_Data[i][1] - My_Current_Point[1], 2));
            Min_Idx2 = i;  
        }
    }
    for (int i = Min_Idx2; i < Obs_Map_Data.size(); i++)
    {
        if (LD < sqrt(pow(Obs_Map_Data[i][0] - My_Current_Point[0], 2) + pow(Obs_Map_Data[i][1] - My_Current_Point[1], 2)))
        {
            My_Current_WayPoint2[0] = Obs_Map_Data[i][0];
            My_Current_WayPoint2[1] = Obs_Map_Data[i][1];
            break;
        }
    }

    double alpha = atan2(My_Current_WayPoint2[0] - My_Current_Point[0], My_Current_WayPoint2[1] - My_Current_Point[1]);
    alpha = alpha - My_Current_Heading / 180.0 * M_PI;
    Delta = atan2(2 * sin(alpha) * 0.38, LD);
    Delta = Delta * 90 /(19.5 * M_PI);
}



void action()
{   
     if(Missin_1_flag)
     {
        Mission_1_Count++;
     }
    if(Min_Idx > 1200 && Min_Idx < 1700)
    {
        if(Mission_1_Static && Mission_1_Count > 100)
        {
            Mission_1_Static = false;
        }
    } 
    if(tmp_flag)
    {
        tmp_cnt++;
        line1 = true;
        line2 = false;
        if(tmp_cnt > 30)
        {
            tmp_flag = false;
            tmp_cnt = 0;
        }
    }
    if(line2 == true && line1 == false && Min_Idx2 < 2700)
    {
        Find_Delta_2();
    }
    else if(line1 == true && line2 == false)
    {
        Find_Delta_1();
    }
    else{
        Find_Delta_1();
    }

    if(Min_Idx > 1200 && Min_Idx < 1800)
    {
        if(Is_Dynamic_Obs_Flag)
        {
            new_flag = true;
        }
    }
    Flag_control();
    if(realparking && parking_cnt ==1)
    {
        int result = std::system((global_path+"/src/kill_node.sh").c_str());
        slam_flag = true;

    }
    if(slam_flag)
    {
        slam_cnt++;
        // cout<<slam_cnt<<endl;
    }
    if(new_flag)
    {
        if(Min_Idx > 1200 && Min_Idx < 1700)
        {
            Find_Delta_1();
        }
    }
    if(slam_cnt > 50)
    {
        if(Static_obs == true && Min_Idx > 3000 && Min_Idx < 4100 && Lidar_XYd[0][0] > 0 && Lidar_XYd[0][0] < 1.3 && abs(Lidar_XYd[0][1]) < 0.4)
        {
            if(Lidar_XYd[0][0] < 0.4) {
                speed_msg_.data = 100;
                slip_lottari = true;
                count_Idx = Min_Idx;

            }
            else {
                speed_msg_.data = 500;
            }
            steer_msg_.data = Delta+0.5;
        }   
        else if(traffic_stop_flag == true)
        {
            speed_msg_.data =0;
            slip_traffic = true;
            count_Idx = Min_Idx;
        }
        else if(Is_Dynamic_Obs_Flag == true)
        {          
            Is_Dynamic_Obs_Count += 1;
            Missin_1_flag = true;
            if(Is_Dynamic_Obs_Count > 1)
            {
                tmp_flag = true;
            }

            speed_msg_.data = 100;
            if(Min_Idx < 2000) { 
                slip_mission2 = true;
                count_Idx = Min_Idx;
            }
            else{
                slip_mission3 = true;
                count_Idx = Min_Idx;
            }
            if(Is_Dynamic_Obs_Count >30)
            {
                Is_Dynamic_Obs_Flag = false;
                Is_Dynamic_Obs_Count = 0;
            }
        }

        else
        {
            speed_msg_.data = 1200;
            steer_msg_.data = Delta+0.5;
            if(Min_Idx >= 3780 && Min_Idx <= 4050)
            {
                speed_msg_.data = 1000;
            }
        }
        speed_pub_.publish(speed_msg_);
        steer_pub_.publish(steer_msg_);
    }
}



void Load_Map()
{
    for (int i = 0; i < 6677; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            if(j == 1)
            {
            path >> Map_Data[i][j];


            }
            else{
            path >> Map_Data[i][j];
            Map_Data[i][j] = Map_Data[i][j] - 0.15;
            }
        }
    }
    for (int i = 0; i < 2833; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            if(j==1)
            {
            path2 >> Obs_Map_Data[i][j];
            Obs_Map_Data[i][j] = Obs_Map_Data[i][j] - 0.1;
            }
            else{
            path2 >> Obs_Map_Data[i][j];

            }
            
        }
    }
}



int main(int argc, char** argv)
{
    Load_Map();
    ros::init(argc, argv, "Control_Node");
    ros::NodeHandle nh;
    gnss_sub_ = nh.subscribe<local_pkg::PosPos>("/PosCal", 10, GNSSCallback);
    lidar_sub_ = nh.subscribe<Lidar_pkg::pointCloud>("/custom_cluster_info", 10, LIDARCallback);
    cam_sub_ = nh.subscribe<dh_pkg::cam_msg>("/Cam_dectection", 10, CAMCallback);
    traffic_sub_ = nh.subscribe<morai_msgs::GetTrafficLightStatus>("/GetTrafficLightStatus", 10, TrafficCallback);
    parking_sub_ = nh.subscribe<wego::parking>("/parking", 10, ParkingCallback);
    speed_pub_ = nh.advertise<std_msgs::Float64>("/commands/motor/speed", 10);  
    steer_pub_ = nh.advertise<std_msgs::Float64>("/commands/servo/position", 10); 
    ros::Rate rate1(50);  
    while (ros::ok())
    {
        ros::spinOnce();
        action();
        rate1.sleep();
    }
    return 0;
}