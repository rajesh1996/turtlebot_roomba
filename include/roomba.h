/**
 * @file roomba.h
 * @author Rajeshwar N S
 * @copyright 2020 Rajeshwar N S
 * @brief Header file containing all the methods and variables declarations
 */

/**
 *MIT License
 *Copyright (c) 2020 Rajeshwar N S
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

#ifndef INCLUDE_ROOMBA_H
#define INCLUDE_ROOMBA_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

/**
 * @class Roomba_bot
 * @brief COntains callback fucntion and laser scan and methods for obstacle avoidance
 *
 */
class Roomba_bot {
public:
    /**
     * @fn  Roomba_bot()
     * @brief Constructor for Roomba_bot
     *
     */
    Roomba_bot();
    /**
     * @fn  ~Roomba_bot()
     * @brief Destructor for Roomba bot
     *
     */
    ~Roomba_bot();
    /**
     * @fn void laserCallback(const sensor_msgs::LaserScan::ConstPtr&)
     * @brief Callback method for laser scan
     *
     * @param msg
     */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    /**
     * @fn bool checkObstacle()
     * @brief Getter for obstacle
     *
     * @return
     */
    bool checkObstacle();
    /**
     * @fn bool setObstacle(bool)
     * @brief Setter function for obstacle
     *
     * @param
     * @return
     */
    bool setObstacle(bool);
    /**
     * @fn void walkRoomba()
     * @brief Method that executes random movement
     *
     */
    void walkRoomba();
private:
    // velocities
    geometry_msgs::Twist msg;
    // node handler
    ros::NodeHandle nh;
    // publish the msg (velocities)
    ros::Publisher pub_vel;
    // subscribe to lidar
    ros::Subscriber sub;
    // check if region in front of robot is obstacle
    bool obstacle;
    // speed for going straight
    float lin_vel;
    // speed for turning
    float ang_vel;
};

#endif // INCLUDE_ROOMBA_HPP
