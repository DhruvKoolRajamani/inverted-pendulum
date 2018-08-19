/**
 ***************************************************
 * PID Controller for a LIP: inverted-pendulum_control_node.h
 * 
 * @author Dhruv Kool Rajamani
 * @version 0.0 19/08/2018
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

/**
 * Initializing global variables
 */

bool                                jointEffortPublish;
bool                                ftSubscriberSet = true;

int                                 n;
int                                 callbackQueueSize = 10;

double                              _tm;

string                              prefix = "/island_1/robot_1";
string                              control_prefix = 
                                        prefix + "/control/config";

geometry_msgs::WrenchStamped        wJoint1;

sensor_msgs::JointState             jointState;

ros::Subscriber                     jointStateSubscriber;
ros::Subscriber                     ftSubscriber;

vector< string >                    jointEffortList;
vector< string >                    ftFeedbackList;
vector< string >                    continuousJointsList;

vector< ros::Publisher >            vec_pubs_efforts;
vector< ros::Subscriber >           vec_ftSensor;

/**
 * Initializing global functions
 */

bool init( ros::NodeHandlePtr node_handle )
{
    ros::Publisher pubs_efforts;

    string jointName;

    for ( int i= 0; i< n; i++ )
    {
        // Initializing vector of strings for joint effort controllers in the spine
        node_handle->getParam(prefix + "/joint/" + to_string(i), jointName);
        continuousJointsList.push_back(jointName);

        cout << i << " : " << continuousJointsList[i] << endl;

        string joint_control_topic_name = 
            control_prefix + "/joint_effort_controller_" + jointName + "/command";
        jointEffortList.push_back(joint_control_topic_name);

        pubs_efforts = node_handle->advertise< std_msgs::Float64 > ( 
                                            jointEffortList[i], 
                                            callbackQueueSize 
                                            );
        vec_pubs_efforts.push_back( pubs_efforts );

        // JointState feedback
        if ( ftSubscriberSet )
        {
            string ftSensorTopicName = prefix + "/ft_sensor/" + jointName;
            ftFeedbackList.push_back( ftSensorTopicName );
        }
    }

    if ( /*condition*/ true )
        return true;
    else
        return false;
}

void jointEffortControllers( double torques[] )
{
    std_msgs::Float64 TmpData;

    int count = 0;

    for ( int i= 0; i< n; i++ )
    {
        TmpData.data = torques[i];
        vec_pubs_efforts[i].publish(TmpData);
        count++;
    }
}

void jointStateCallback( const sensor_msgs::JointState::ConstPtr & msg )
{
    // jointState = *msg;
    ROS_INFO( "Position: %f", msg->position[0] );
    cout << msg->position[0] << endl;
}

void jointStateFeedback( ros::NodeHandlePtr node_handle )
{
    string topic = prefix + "/joint_states";
    jointStateSubscriber = node_handle->subscribe( 
                            topic,
                            callbackQueueSize, 
                            jointStateCallback
                            );
}

void ftCallback( const geometry_msgs::WrenchStamped& msg )
{
    // ROS_INFO( "stamp.sec: %d", msg.header.stamp.sec );
    // wJoint1 = *msg;

}

// Should be made async-> on another thread
void ftFeedback( ros::NodeHandlePtr node_handle )
{
    if ( ftSubscriberSet )
    {
        for ( int i= 0; i< ftFeedbackList.size(); i++ )
        {
            // cout << f3dFeedbackLegsList[i] << " : " << endl;
            ftSubscriber = node_handle->subscribe( 
                                ftFeedbackList[i].c_str(), 
                                callbackQueueSize, 
                                ftCallback
                                );
        }
    }
}

double getSimulationTime()
{
    _tm = ros::Time::now().toSec();
    return _tm;
}