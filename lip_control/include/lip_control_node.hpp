/**
 ***************************************************
 * PID Controller for a LIP: lip_control_node.h
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

static int                          n;

int                                 callbackQueueSize = 10;
int                                 islands, robots;

double                              _tm;

string                              prefix = "/island_1/robot_1/";
string                              control_prefix = 
                                        prefix + "control/config/";

geometry_msgs::WrenchStamped        wJoint1;

sensor_msgs::JointState             jointState;

ros::Subscriber                     jointStateSubscriber;
ros::Subscriber                     ftSubscriber;

vector< string >                    jointEffortList;
vector< string >                    ftFeedbackList;
vector< string >                    continuousJointsList;
vector< string >                    vPrefix;

vector< float >                     jsPos;
vector< float >                     jsVel;

vector< double >                    torques;

vector< ros::Publisher >            vec_pubs_efforts;
vector< ros::Subscriber >           vec_ftSensor;

/**
 * Initializing global functions
 */

void setParameters( ros::NodeHandlePtr node_handle )
{
    node_handle->getParam(
        "islands",
        islands
    );
    cout << islands;
    for ( int i= 1; i< islands + 1; i++ )
    {
        node_handle->getParam(
        "islands_" + to_string(i) + "/robots",
        robots
        );

        for ( int j= 1; j< robots + 1; j++ )
        {
            node_handle->getParam(
                "namespace_" + to_string(i) + "_" + to_string(j),
                prefix
            );
            vPrefix.push_back( prefix );
        }
    }

    node_handle->getParam(
        prefix + "n_joints",
        n
    );
}

bool init( ros::NodeHandlePtr node_handle )
{
    ros::Publisher pubs_efforts;

    string jointName;

    for ( int i= 0; i< n; i++ )
    {
        // Initializing vector of strings for joint effort controllers
        node_handle->getParam(
            prefix + "joint/" + 
            to_string(i), 
            jointName
            );
        continuousJointsList.push_back(jointName);

        // cout << i << " : " << continuousJointsList[i] << endl;

        string joint_control_topic_name = 
            control_prefix + "joint_effort_controller_" 
            + jointName + "/command";
        jointEffortList.push_back(joint_control_topic_name);

        pubs_efforts = node_handle->advertise< std_msgs::Float64 > ( 
                                            jointEffortList[i], 
                                            callbackQueueSize 
                                            );
        vec_pubs_efforts.push_back( pubs_efforts );

        // JointState feedback
        if ( ftSubscriberSet )
        {
            string ftSensorTopicName = prefix + "ft_sensor/" + jointName;
            ftFeedbackList.push_back( ftSensorTopicName );
        }

        jsPos.push_back( 0.0 );
        jsVel.push_back( 0.0 );

        torques.push_back( 0.0 );
    }

    if ( /*condition*/ true )
        return true;
    else
        return false;
}

void jointEffortControllers( vector< double > torques )
{
    std_msgs::Float64 TmpData;

    for ( int i= 0; i< n; i++ )
    {
        TmpData.data = torques[i];
        vec_pubs_efforts[i].publish(TmpData);
    }
}

void jointStateCallback(
    const sensor_msgs::JointState::ConstPtr & msg
    )
{
    jointState = *msg;
}

void jointStateFeedback( ros::NodeHandlePtr node_handle )
{
    string topic = prefix + "joint_states";
    jointStateSubscriber = node_handle->subscribe( 
                            topic,
                            n, 
                            jointStateCallback
                            );
}

void setStateFeedback()
{
    for ( int i= 0; i< n; i++ )
    {
        for ( int j = 0; j< jointState.name.size(); j++ )
        {
            if (( 
                continuousJointsList[i].compare( 
                                    jointState.name[j]
                                    ) 
                            ) 
                == 0 )
            {
                jsPos[i] = (float)jointState.position[j];
                jsVel[i] = (float)jointState.velocity[j];

                cout << i << " : " << jsPos[i] << endl;
            }
        }
    }
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

void converge()
{
    double y[n];  // desired output to be driven to zero
    double dy[n]; // derivative of the desired output to be driven to zero
    double pos_des;
    double vel_des;
    double Kp[n], Kd[n];
    double ALPHA = 0.3;

    double Qfinal[n];

    for( int i= 0; i< n; i++ )
    {
        Qfinal[i] = 0.0;
        Kp[i] = 100;
        Kd[i] = 10;
    }

    for ( int i = 0; i< n; i++ )
    {
        y[i] = Qfinal[i]*( 1 + exp( -ALPHA * _tm ) ) - jsPos[i];
        dy[i] = - ALPHA*Qfinal[i]*exp( -ALPHA * _tm ) - jsVel[i];
        
        /* Uncomment for debugging */
        // cout << _tm << " : " << i << " : " << Pos_sens[i] << " : " 
        //      << Pos_sens0[i] << " : " << Qfinal[i] << " : " 
        //      << pos_des << " : " << tauDes[i] << endl;
    }

    for ( int i= 0; i< n; i++ )
    {
        double temp;
        temp = Kp[i]*(y[i]) + Kd[i]*(dy[i]);
        if( temp < 10000000000000 && temp > -10000000000000 )
            torques[i] = temp;
        else
        {
            torques[i] = 0.0;
        }
    }
}

double getSimulationTime()
{
    _tm = ros::Time::now().toSec();
    return _tm;
}