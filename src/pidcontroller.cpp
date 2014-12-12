

#include <pidcontroller.h>




/* Implementing Feedback control laws */
VectorXd control_model::getControl (VectorXd statevector, double reference_position, double position)
{
    VectorXd control;

    control.resize (nDOF_);
    double error_component = reference_position - position;
    //control(0) = 0.5 * 0;
    Vector3d coefficient;
    coefficient(0) = _p;
    coefficient(1) = _i;
    coefficient(2) = _d;
    control = pid(coefficient, error_component);
                        
    
    return control;
}
        
/* Setting coeeffecient of PID controller  */
void pid_controller::setcontrollerparameter(int p, int i, int d)
{
    _p = p;
    _i = i;
    _d= d;
    
}
        
VectorXd pid_controller::pid(Vector3d coeffecient, double error_component)
{
    VectorXd control;
            
    control = coeffecient * error_component;
            
    return control;
}