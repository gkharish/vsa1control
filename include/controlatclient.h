#ifndef CONTROLATCLIENT_H
#define CONTROLATCLIENT_H

//#include "serverudp3.h"
#include "clientudp3.h"

#include <Eigen/Core>
#include <math.h>

#define GRAVITY 9.81
#define pi 3.14
using namespace std;
using namespace Eigen;

class  control_model :    public ClientUDP   
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
            control_model () : ClientUDP()
            {
                    
            }
                
            void setpidcoeff(int p, int i, int d);
            //MatrixXd trajecotystore(MatrixXd reference_traj, MatrixXdstate_traj, MatrixXd control_traj, int col);
            //MatrixXd trajectorystore(VectorXd traj, int column);
            VectorXd pid(Vector3d coefficient, double error_component );
            VectorXd getControl (VectorXd statevector, double reference_position, double position);
                
};
    
#endif
