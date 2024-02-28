#define PI 3.14159265358979323846

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <sensor_msgs/Imu.h>  //imu -> heading   vx vy 분해 가능 이걸 x 시간 = 이동 거리 
#include <local_pkg/VescStateStamped.h>      //(0,0) 
#include <local_pkg/VescState.h>  //현재 속도
#include <local_pkg/PosPos.h>		
// #include <morai_msgs/EgoVehicleStatus.h>
#include <typeinfo>


using namespace std;

double yaw = 0;
double vel = 0;

ros::Duration dt;
ros::Time pre_t, cur_t;

bool signal_t = false;

vector<double> Vel(2, 0);
vector<double> Pos(2, 0);

local_pkg::PosPos msgg;
ros::Publisher pub;

double Deg2Rad(double Deg)
{
	return (Deg / 180 * PI);
}
float Deg2Rad(float Deg)
{
	return (Deg / 180 * PI);
}

double Rad2Deg(double Rad)
{
	return (Rad / PI * 180);
}
float Rad2Deg(float Rad)
{
	return (Rad / PI * 180);
}

void rcv_IMU(const sensor_msgs::Imu::ConstPtr &IMU)
{
	double t4 = 2 * (IMU->orientation.w * IMU->orientation.z + IMU->orientation.x * IMU->orientation.y);
	double t5 = 1 - 2 * (pow(IMU->orientation.y, 2) + pow(IMU->orientation.z, 2));

	yaw = Rad2Deg(atan2(t4, t5)); // NED
}

void rcv_Velocity(const local_pkg::VescStateStamped::ConstPtr &Velocity)
{
	if (signal_t == false)
	{
		pre_t = ros::Time::now();
		signal_t = true;
	}
	else
	{
		cur_t = ros::Time::now();
		dt = cur_t - pre_t;
		pre_t = cur_t;
	}
	vel = Velocity->state.speed * 0.0013577451514397 * 1000 / (2 * PI * 60);
	Vel[0] = vel * cos(Deg2Rad(yaw)); //x방향속도
	Vel[1] = vel * sin(Deg2Rad(yaw));  //y방향속도
	vector<double> dPos(2, 0);
	dPos[0] = dt.toSec() * Vel[0] / 3.6;  //x로 간 거리 [m]  
	dPos[1] = dt.toSec() * Vel[1] / 3.6;  //y로 간 거리 [m]
	Pos[0] -= dPos[1];
	Pos[1] += dPos[0];
	cout << "----------------------------------------" << endl;
	cout << "VelX : " << Vel[0] << "   VelY : " << Vel[1] << "   total Vel : " << vel << endl;
	cout << "yaw : " << yaw << "   velocity : " << vel << "   vel2 : " << sqrt(pow(Vel[0], 2) + pow(Vel[1], 2)) << endl;
	cout << "dt : " << dt.toSec() << endl;
	cout << "cal_posX : " << Pos[0] << "   cal_posY : " << Pos[1] << endl;
	msgg.PosX = Pos[0];
	msgg.PosY = Pos[1];
	msgg.heading = yaw;
	
}
int main(int argc, char **argv)
{
	Pos[0] = 0;
	Pos[1] = 0;
	// 코드가 제대로 안돌고 다른 노드들한테 간섭 안주고(그 코드가 돌아서 이 코드가 느려지는걸) -> 
	// 제어 핸들, 속도 -> (우리 패스에대해)
	ros::init(argc, argv, "Control");
	ros::NodeHandle nh;
	ros::Subscriber sub2 = nh.subscribe<sensor_msgs::Imu>("/imu", 1, rcv_IMU);
	ros::Subscriber sub3 = nh.subscribe<local_pkg::VescStateStamped>("/sensors/core", 1, rcv_Velocity);
	// ros::Subscriber sub4 = nh.subscribe<morai_msgs::EgoVehicleStatus>("/Ego_topic", 1, rcv_Ego);
	pub = nh.advertise<local_pkg::PosPos>("/PosCal", 1);
	ros::Rate rate(50);

	while(ros::ok())
	{
		ros::spinOnce();
		pub.publish(msgg);
		rate.sleep();
	}
	ros::spin();
}