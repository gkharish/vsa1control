

#include <pidcontroller.h>




pid_controller::pid_controller()
{
   
    
}

        
/* Setting coeeffecient of PID controller  */
void pid_controller::setpidcoeff(int p, int i, int d)
{
    _p = p;
    _i = i;
    _d= d;
    coefficient << _p, _i, _d;
    
}
        
VectorXd pid_controller::pid( double error_component)
{
    VectorXd control;
            
    control = coefficient * error_component;
            
    return control;
}