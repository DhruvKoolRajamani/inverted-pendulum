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

    NodeHandlePtr nhMain = 
        ros::NodeHandlePtr( new ros::NodeHandle( "~" ) );

    Rate loop_rate = 1000;

    setParameters( nhMain );
    
    if( init( nhMain ) )
    {
        jointStateFeedback( nhMain );
        ftFeedback( nhMain );

        while ( nhMain->ok() )
        {
            _tm = getSimulationTime();
            
            setStateFeedback();

            if( !bBegin )
            {
                init_pos();
                if( !converged )
                {
                    jointEffortControllers( torques );
                    bBegin = true;
                }
            }

            converge();

            if ( !converged )
                jointEffortControllers( torques );
            else
                cout << "\nConverged\n";

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    

    return 0;
}