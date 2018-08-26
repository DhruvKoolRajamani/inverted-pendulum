#include "lip_controller.hpp"

lip_controller::lip_controller( ros::NodeHandlePtr node_handle )
{
    nhLIP = node_handle;

    ros::Publisher pubs_efforts;

    string jointName;

    for ( int i= 0; i< n; i++ )
    {
        // Initializing vector of strings for joint effort controllers
        nhLIP->getParam(
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

        pubs_efforts = nhLIP->advertise< std_msgs::Float64 > ( 
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
}

void lip_controller::setParameters()
{
    nhLIP->getParam(
        "islands",
        islands
    );
    cout << islands;
    for ( int i= 1; i< islands + 1; i++ )
    {
        nhLIP->getParam(
        "islands_" + to_string(i) + "/robots",
        robots
        );

        for ( int j= 1; j< robots + 1; j++ )
        {
            nhLIP->getParam(
                "namespace_" + to_string(i) + "_" + to_string(j),
                prefix
            );
            vPrefix.push_back( prefix );
        }
    }

    nhLIP->getParam(
        prefix + "n_joints",
        n
    );
}

void lip_controller::jointEffortControllers( vector< double > torques )
{
    std_msgs::Float64 TmpData;

    for ( int i= 0; i< n; i++ )
    {
        TmpData.data = torques[i];
        vec_pubs_efforts[i].publish(TmpData);
    }
}

void lip_controller::jointStateCallback(
    const sensor_msgs::JointState::ConstPtr & msg
    )
{
    jointState = *msg;
}

void lip_controller::jointStateFeedback()
{
    string topic = prefix + "joint_states";
    jointStateSubscriber = nhLIP->subscribe(
                            topic,
                            callbackQueueSize, 
                            &lip_controller::jointStateCallback,
                            this);
}

void lip_controller::setStateFeedback()
{
    lip_controller::jointStateFeedback();
    for ( int i= 0; i< n; i++ )
    {
        for ( int j = 0; j< jointState.name.size(); j++ )
        {
            if (( continuousJointsList[i].compare( 
                                    jointState.name[j] )) 
                                                    == 0 )
            {
                jsPos[i] = (float)jointState.position[j];
                jsVel[i] = (float)jointState.velocity[j];
            }
        }
    }
}

void lip_controller::getStateFeedback( vector< float > _qSens, vector< float > _dqSens )
{
    lip_controller::setStateFeedback();

    for ( int i = 0; i< n; i++ )
    {
        _qSens[i] = jsPos[i];
        _dqSens[i] = jsVel[i];
    }
}

// void ftCallback( const geometry_msgs::WrenchStamped& msg )
// {
//     // ROS_INFO( "stamp.sec: %d", msg.header.stamp.sec );
//     // wJoint1 = *msg;

// }

// // Should be made async-> on another thread
// void ftFeedback( ros::NodeHandlePtr nhLIP )
// {
//     if ( ftSubscriberSet )
//     {
//         for ( int i= 0; i< ftFeedbackList.size(); i++ )
//         {
//             // cout << f3dFeedbackLegsList[i] << " : " << endl;
//             ftSubscriber = nhLIP->subscribe( 
//                                 ftFeedbackList[i].c_str(), 
//                                 callbackQueueSize, 
//                                 ftCallback
//                                 );
//         }
//     }
// }

double lip_controller::getSimulationTime()
{
    _tm = ros::Time::now().toSec();
    return _tm;
}

int lip_controller::getNLinks()
{
    return n;
}