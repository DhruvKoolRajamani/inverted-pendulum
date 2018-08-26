#include <iostream>
#include <math.h>
#include <vector>

void converge(
    const int n, const double _tm, 
    std::vector< float > jsPos, 
    std::vector< float > jsVel, 
    std::vector< double > torques
    )
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
        std::cout << _tm << " : " << i << " : " << jsPos[i] << " : " 
             << y[i] << " : " << jsVel[i] << " : " 
             << dy[i] << " : " << std::endl;
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