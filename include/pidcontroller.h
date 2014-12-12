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
            int nDOF_;
            //MatrixXd reference_traj, state_traj, control_traj;
            
    public:
            /// Constructor
            pid_controller();
            
                
            void setcontrollerparameter(int p, int i, int d);
            //void setsmcparameters(double lambda, double phi, double gain);
            //MatrixXd trajecotystore(MatrixXd reference_traj, MatrixXdstate_traj, MatrixXd control_traj, int col);
            //MatrixXd trajectorystore(VectorXd traj, int column);
            VectorXd pid(Vector3d coefficient, double error_component );
            //VectorXd getControl (VectorXd statevector, double reference_position, double position);
                
};
    
#endif
