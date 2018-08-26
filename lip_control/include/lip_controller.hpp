/**
 ***************************************************
 * ROS Interface for a LIP: lip_controller.hpp
 * 
 * @author Dhruv Kool Rajamani
 * @version 0.0 25/08/2018
 ***************************************************
 */

#pragma once

#include <iostream>
#include <string>
#include <time.h>
#include <math.h>
#include <vector>
#include <utility>

#include "ros/ros.h"
#include "ros/spinner.h"
#include "ros/callback_queue.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/Imu.h"

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

using namespace std;
using namespace ros;

class lip_controller
{
    /***
     * Public Members
     */
    public:
    bool                                ftSubscriberSet = true;

    double                              _tm;

    vector< float >                     jsPos;
    vector< float >                     jsVel;

    vector< double >                    torques;

    /***
     * Public Member Functions
     */
    lip_controller( ros::NodeHandlePtr node_handle );

    void setParameters();
    
    void jointEffortControllers( vector< double > torques );
    
    void jointStateCallback( const sensor_msgs::JointState::ConstPtr& );
    void jointStateFeedback();
    void setStateFeedback();
    void getStateFeedback( vector< float > _qSens, vector< float > _dqSens );

    double getSimulationTime();
    int getNLinks();

    /***
     * Private Members
     */
    private:
    bool                                jointEffortPublish;

    int                                 islands, robots;
    
    int                                 n = 2;

    const int                           callbackQueueSize = 10;

    string                              prefix = "/island_1/robot_1/";
    string                              control_prefix = 
                                            prefix + "control/config/";

    geometry_msgs::WrenchStamped        wJoint1;

    sensor_msgs::JointState             jointState;

    ros::NodeHandlePtr                  nhLIP;

    ros::Subscriber                     jointStateSubscriber;
    ros::Subscriber                     ftSubscriber;

    vector< string >                    jointEffortList;
    vector< string >                    ftFeedbackList;
    vector< string >                    continuousJointsList;
    vector< string >                    vPrefix;

    vector< ros::Publisher >            vec_pubs_efforts;
    vector< ros::Subscriber >           vec_ftSensor;

    /***
     * Private Member Functions
     */
};