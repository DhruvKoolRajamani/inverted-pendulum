/**
 ***************************************************
 * PID Controller for a LIP: pid_control.cpp
 * 
 * @author Dhruv Kool Rajamani
 * @version 0.0 25/08/2018
 ***************************************************
 */

#include "lip_controller.hpp"
#include "pid_control/pid_converge.cpp"

int                                             n;

double                                          _tm;
            
std::vector< float >                            jsPos;
std::vector< float >                            jsVel;

std::vector< double >                           torques;

void init()
{
    for ( int i= 0; i< n; i++ )
    {
        jsPos.push_back( 0.0 );
        jsVel.push_back( 0.0 );

        torques.push_back( 0.0 );
    }
}

int main( int argc, char** argv )
{
    ros::init( argc, argv, "pid_control" );

    NodeHandlePtr nhMain = 
        ros::NodeHandlePtr( new ros::NodeHandle( "~" ) );

    ros::Rate loop_rate = 1000;
    
    lip_controller controller( nhMain );

    // controller.setParameters();

    n = controller.getNLinks();

    init();

    // controller.ftFeedback();

    while ( nhMain->ok() )
    {
        _tm = controller.getSimulationTime();
        
        controller.setStateFeedback( jsPos, jsVel );

        converge(
            n,
            _tm,
            jsPos,
            jsVel,
            torques
        );

        controller.jointEffortControllers( torques );

        // std::cout << _tm << " : " << n << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }    

    return 0;
}