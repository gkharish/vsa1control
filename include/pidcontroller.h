#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

//#include "clientudp3.h"
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fstream>
#include <sstream>
#include <Eigen/Core>
#include <math.h>

#define GRAVITY 9.81
#define pi 3.14
using namespace std;
using namespace Eigen;

class  pid_controller   
{
    protected:
            
            //PID controller parameters
            double _p;
            double _i;
            double _d;
            Vector3d coefficient;
            int nDOF_;
            //MatrixXd reference_traj, state_traj, control_traj;
            
    public:
            /// Constructor
            pid_controller();
            
                
            void setpidcoeff(int p, int i, int d);
            
            VectorXd pid(double error_component );
            
                
};
    
#endif
