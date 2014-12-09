#include "controlatclient.h"
//#include "clientudp3.h"
//#include "plantmodel.h"    

#include <native/task.h>
#include <native/timer.h>

#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fstream>
#include <sstream>



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
void control_model::setpidcoeff(int p, int i, int d)
{
    _p = p;
    _i = i;
    _d= d;
    
}
        
VectorXd control_model::pid(Vector3d coeffecient, double error_component)
{
    VectorXd control;
            
    control = coeffecient * error_component;
            
    return control;
}
        
/*MatrixXd trajectorystore(VectorXd traj, int column)
{
    MatrixXd trajstored;
    trajstored.col(column) = traj;
    return trajstored;
}*/
        
        

/* REFERENCE GENERATOR FUNCTION */

double reference_generator(double ref_pos)
{
    //double reference_position = ref_pos;
            
    return ref_pos*pi/180;
            
}
        
// Packets to be sent //    
 
struct udppacket_control
{
    char CLIENT_HEADER;
    unsigned int control_cmd[3];
}client_packet_control;
    
struct udppacket_bool
{
    char CLIENT_HEADER;
    bool data;
}client_packet_bool;
   
// packets to be received //
struct udppacket_DAQ
{
    char SERVER_HEADER;
    float data[32];
}client_packet_DAQ;  
    
struct udppacket_COUNTER
{
    char SERVER_HEADER;
    signed int data[12];
}client_packet_COUNTER;  
 
std::ostream& operator<<(std::ostream& os, const struct udppacket_control & obj)
{
    // write obj to stream
     os << " " << obj.CLIENT_HEADER 
	<< " " << obj.control_cmd[0] 
	<< " " << obj.control_cmd[1] 
	<< " " << obj.control_cmd[2];
    return os; 
}  
    
std::ostream& operator<<(std::ostream& os, const struct udppacket_bool & obj)
{
    // write obj to stream
    os << " " << obj.CLIENT_HEADER 
	<< " " << obj.data; 
	 
    return os; 
}          

std::ostream& operator<<(std::ostream& os, const struct udppacket_DAQ & obj)
{
    // write obj to stream
    os << " " << obj.SERVER_HEADER 
    << " " << obj.data[0] 
    << " " << obj.data[1] 
    << " " << obj.data[2]
    << " " << obj.data[3];
    return os; 
}  
    
std::ostream& operator<<(std::ostream& os, const struct udppacket_COUNTER & obj)
{
    // write obj to stream
    os << " " << obj.SERVER_HEADER 
    << " " << obj.data[0] 
    << " " << obj.data[1]; 
    return os; 
}        
        
        
        
/* Real Time function  */
RT_TASK principal_task;
MatrixXd outputprint;
        
/*  Variables used in data storage */
MatrixXd reference_traj, state_traj, control_traj;

            
            
            
        /* *****     PRINCIPAL FUNCTION       ***** */

void principal_function(void *argv)
{
    
            
    
    /* variables used in the principal program */
    int whileloop_counter = 0, error_counter = 0, loop = 0;
    int timeofsimulation_s = 30; /* time in seconds*/
    int FLAG = 1;
    int nsig;
        
    /* Variables of system dynamics state space */
    VectorXd initial_state(4); 
    initial_state << 0, 0, 0, 0;

    /*  Variables used in Controller */
    int p = 10;
    int i = 1;
    int d = 1;
    VectorXd u(3);
    VectorXd initial_control(3);
    initial_control << 1.3, 2.01,0.001;
    //u(0) = u_init;
    //VSA1axis -> setpidcoeff(p,i,d);
            
    /*  Variables used in UDP communication */        
    char buf[BUFLEN];
    
    udppacket_control send_packet;
    udppacket_bool send_packet_bool;
    char*  buffer_send;
  
    udppacket_COUNTER * recv_packet_COUNTER;
    udppacket_DAQ * recv_packet_DAQ;  
    char recv_buffer[BUFLEN];
    
    /*  Variables used in real time Timer   */     
    RTIME  now, previous, TASK_PERIOD = 1000000;
    double t, time_start_loop, present_time; 
    int numsamples_s = (1e9/TASK_PERIOD); 
    
    /* Variables used in reference generator */
    double ref_pos = 30;
    VectorXd ref_init(4);
    ref_init << 0.52,0,0.52,0;
    cout << ref_init;
        
    /*  Initialization */
            
            
            
    control_model* PID_control = new control_model(); // creating an object of pam 
    PID_control -> client_start();
            
    PID_control -> setpidcoeff(p,i,d);
    u << initial_control;
    VectorXd previous_state = initial_state;
            
    /* initializing data storage*/
    state_traj.resize (timeofsimulation_s*numsamples_s, initial_state.size() );
    state_traj.row(0) = initial_state;
            
    reference_traj.resize(timeofsimulation_s*numsamples_s, ref_init.size());
    reference_traj.row(0) = ref_init;
            
    control_traj.resize(timeofsimulation_s*numsamples_s, u.size());
    control_traj.row(0) =  u;
            
            
    rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD));
            
    now = rt_timer_read();
    time_start_loop  = round(now/1.0e9);
    
            
            
    /* ** sending controls  **** */ 
    int control_len, control_size, state_len, states_size;
    control_size= u.size();
    states_size = previous_state.size();
            
            /**/
    
    send_packet.CLIENT_HEADER = '0';
    send_packet.control_cmd[0] = u(0);
    send_packet.control_cmd[1] = u(1);
    send_packet.control_cmd[2] = u(2);
    buffer_send = (char*)&send_packet;
    
    //struct udppacket_control *asp_control = &send_packet;
    //std::cout << "\n  server message received is unsigned int: " << *asp_control << std::endl;
    PID_control -> client_send(buffer_send, sizeof(send_packet));
    
    

    while(FLAG)
    {
        rt_task_wait_period(NULL);
                
        now = rt_timer_read();
        present_time  = round(now/1.0e9);
        t = present_time - time_start_loop;
        double reference_position = reference_generator(ref_pos);
        
        
        /* ** Recive data from server ** */
        PID_control -> client_recv(recv_buffer, BUFLEN);
    		
    	
    	switch(recv_buffer[0])
    	{
    		    
    	    case '0' :
    		{
    		        
    		    recv_packet_DAQ = (udppacket_DAQ *)recv_buffer;
    		        
        	    //std::cout << "\n  server message received is DAQ float type: " << *recv_packet_DAQ << std::endl;
        	    break;
    	    }
    		    
    		    
    		    
            case '1' :
    	    {
    		    recv_packet_COUNTER = (udppacket_COUNTER *)recv_buffer;
    		        
        	    //std::cout << "\n  server message received is COUNTER of signed int type : " << *recv_packet_COUNTER << std::endl;
        	    break;
            }
    		    
    		    
    		    
    		    
    		   
    	}
        
        previous_state << (*recv_packet_DAQ).data[0], (*recv_packet_DAQ).data[1], 
                          (*recv_packet_DAQ).data[2], (*recv_packet_DAQ).data[3];
        
        
        cout << "\n Previous state \n" << previous_state;
        double position = previous_state(2);
        
        u = PID_control -> getControl(previous_state, reference_position, position );

        /* ** Clinet send control data ** */
        
        send_packet.CLIENT_HEADER  = '0';
        send_packet.control_cmd[0] = u(0);
        send_packet.control_cmd[1] = u(1);
        send_packet.control_cmd[2] = u(2);
        buffer_send = (char*)&send_packet;
        //cout << "\n after buffer load and before client send :" << buffer_send;
        struct udppacket_control *asp_control = &send_packet;
        std::cout << "\n  client message send is unsigned int: " << *asp_control << std::endl;
        PID_control -> client_send(buffer_send, sizeof(send_packet));
                
        /* Data storage */
         whileloop_counter++;
        reference_traj.row(whileloop_counter) << 0, 0, reference_position, 0;
        state_traj.row(whileloop_counter)(2) = position;
        control_traj.row(whileloop_counter) = u;
        /*Data storage ends*/
                
        /*state_traj = trajectorystore(previous_state, whileloop_counter);
        control_traj = trajectorystore(u, whileloop_counter); 
        reference_traj = trajectorystore(ref_init, whileloop_counter);*/
                
        //whileloop_counter++;
        cout << "/n the time past is : " << t;
        if(t >= timeofsimulation_s)
        {
            FLAG = 0;
            //printf("\n Task completion: \n\n\n");
            cout << "\n END of Loop";

        }
    } 
            
}
        
        /*** SiGNAL catch  **/
         
void catch_signal(int sig)
{
        
}
        
        
        
        /* ****  MAIN FUNCTION **** */
        
        
int main (int argc, char* argv[])
{
            
    int n ;
    //FILE* statetrajdata, referencetrajdata, controldata;
    //char* path = "/usr/users/localuser/softdev/vsa1control/data/state_trajectory.xls"; 
            
    ofstream statetrajdata, referencetrajdata, controltrajdata;
    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);
    char* a;
    mlockall(MCL_CURRENT|MCL_FUTURE);
	        
    

    n = rt_task_create(&principal_task, "principal_function", 0, 99, 0);
    if (n!=0)
    {
        cout << "Failed @ RT Create" << endl;
    }
    else cout << "END of RT Create" << endl;
    /*
    * Arguments: &task,
    *            task function,
    *            function argument
    */
    n = rt_task_start(&principal_task, &principal_function, NULL);
    if (n!=0)
    {
        cout << "Failed of RT STart" << endl;
    }
    else cout << "END of RT Start" << endl;
    pause();

    cout << "END of Pause" << endl;

        
    n = rt_task_delete(&principal_task);
    if(n!=0)cout << "Failed of RT Task delete" << endl;
    else cout << "END of RT taslk delete";
            
            /*  Trajecory storage*/
    statetrajdata.open ("state_trajectory.txt");
    statetrajdata << state_traj ;//"Writing this to a file.\n";
    statetrajdata.close();
            
    referencetrajdata.open ("reference_trajectory.txt");
    referencetrajdata << reference_traj ;//"Writing this to a file.\n";
    referencetrajdata.close();
            
    controltrajdata.open ("control_trajectory.txt");
    controltrajdata << control_traj ;//"Writing this to a file.\n";
    controltrajdata.close();
            
    /*cout << "\n rows and column of state_traj: " << state_traj.rows() << "," << state_traj.cols();
    cout << "\n rows and column of ref_traj: " << reference_traj.rows() << "," << reference_traj.cols();
    cout << "\n rows and column of control_traj: " << control_traj.rows() << "," << control_traj.cols();*/
            
    /* cout << "\n last column of state_traj : " << state_traj.col(3);
    cout << "\n last column of ref_traj : " << reference_traj.col(3);
    cout << "\n last column of control_traj : " << control_traj.col(3);*/
    //cout << state_traj << endl;
            
}
             
