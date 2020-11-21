#ifndef TURTLEBOT_ROOMBA_ROOMBA_HPP
#define TURTLEBOT_ROOMBA_ROOMBA_HPP

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

class Roomba_bot {
public:
    Roomba_bot();
    ~Roomba_bot();
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    bool checkObstacle();
    void walkRoomba();
private:
    geometry_msgs::Twist msg;
    ros::NodeHandle nh;
    ros::Publisher pub_vel;
    ros::Subscriber sub;
    bool obstacle;
    float lin_vel;
    float ang_vel;
};



#endif //TURTLEBOT_ROOMBA_ROOMBA_HPP
