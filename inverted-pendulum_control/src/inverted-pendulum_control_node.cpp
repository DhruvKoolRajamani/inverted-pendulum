/**
 ***************************************************
 * PID Controller for a LIP: inverted-pendulum_control_node.cpp
 * 
 * @author Dhruv Kool Rajamani
 * @version 0.0 19/08/2018
 ***************************************************
 */

#include "inverted-pendulum_control_node.hpp"

int main( int argc, char** argv )
{
    ros::init( argc, argv, "PID_Control" );

    ros::NodeHandlePtr nhMain = 
        ros::NodeHandlePtr( new ros::NodeHandle( "/PID_Control" ) );

    ros::Rate loop_rate = 1000;

    n = 1;
    
    jointStateFeedback( nhMain );
    ftFeedback( nhMain );

    while ( nhMain->ok() )
    {
        _tm = getSimulationTime();

        // cout << _tm << " : " << jointState.position[0] << " : " << endl;

        // cout << _tm << endl;
    }
    

    return 0;
}