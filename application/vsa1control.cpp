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
 
struct udppacket_control                    // clientheader = '0';
{
    char CLIENT_HEADER;
    double control_cmd[3];
    //unsigned int control_cmd[16];
}client_packet_control;
    
struct udppacket_countersreset              // clientheader = '1';
{
    char CLIENT_HEADER;
    bool data;
}client_packet_countersreset;

struct udppacket_digitaloutputcontrol       // clientheader = '2';
{
    char CLIENT_HEADER;
    bool data;
}client_packet_digitaloutputcontrol;
   
// packets to be received //
struct udppacket_DAQ                        // serverheader = 'a';
{
    char SERVER_HEADER;
    float data[32];
}client_packet_DAQ;  
    
struct udppacket_COUNTER                    // serverheader = 'b';
{
    char SERVER_HEADER;
    signed int data[12];
}client_packet_COUNTER;  

struct udppacket_error                      // serverheader = 'c';
{
    char SERVER_HEADER;
    unsigned char data[4];
}client_packet_error; 
 
std::ostream& operator<<(std::ostream& os, const struct udppacket_control & obj)
{
    // write obj to stream
     os << " " << obj.CLIENT_HEADER 
	<< " " << obj.control_cmd[0] 
	<< " " << obj.control_cmd[1] 
	<< " " << obj.control_cmd[2];
    return os; 
}  
    
std::ostream& operator<<(std::ostream& os, const struct udppacket_countersreset & obj)
{
    // write obj to stream
    os << " " << obj.CLIENT_HEADER 
	<< " " << obj.data; 
	 
    return os; 
}  

std::ostream& operator<<(std::ostream& os, const struct udppacket_digitaloutputcontrol & obj)
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
    << " " << obj.data[3]
    << " " << obj.data[4];
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
        
std::ostream& operator<<(std::ostream& os, const struct udppacket_error & obj)
{
    // write obj to stream
    os << " " << obj.SERVER_HEADER 
    << " " << obj.data[0] 
    << " " << obj.data[1]
    << " " << obj.data[2]
    << " " << obj.data[3];
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
    int timeofsimulation_s = 5; /* time in seconds*/
    int FLAG = 1;
    int nsig;
        
    /* Variables of system dynamics state space */
    VectorXd initial_state(5); 
    initial_state << 0.01, 0, 0, 101e3, 101e3;

    /*  Variables used in Controller */
    int p = 1;
    int i = 1;
    int d = 1;
    VectorXd u(3);
    VectorXd initial_control(3);
    initial_control << 0.1, 0, 0;
    //u(0) = u_init;
    //VSA1axis -> setpidcoeff(p,i,d);
            
    /*  Variables used in UDP communication */        
    char buf[BUFLEN];
    
    udppacket_control send_packet;
    udppacket_countersreset send_packet_countersreset;
    udppacket_digitaloutputcontrol send_packet_digitaloutputcontrol;
    char*  buffer_send;
  
    udppacket_COUNTER * recv_packet_COUNTER;
    udppacket_DAQ * recv_packet_DAQ; 
    udppacket_error * recv_packet_error;
    char recv_buffer[BUFLEN];
    
    /*  Variables used in real time Timer   */     
    RTIME  now, previous, TASK_PERIOD = 1000000;
    double t, time_start_loop, present_time; 
    int numsamples_s = (1e9/TASK_PERIOD); 
    
    /* Variables used in reference generator */
    double ref_pos = 30;
    VectorXd ref_init(4);
    ref_init << 0.52,0,0.52,0;
    //cout << ref_init;
        
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
    		    
    	    case 'a' :
    		{
    		        
    		    recv_packet_DAQ = (udppacket_DAQ *)recv_buffer;
    		    
    		    previous_state << (*recv_packet_DAQ).data[0], (*recv_packet_DAQ).data[1], 
                          (*recv_packet_DAQ).data[2], (*recv_packet_DAQ).data[3],
                          (*recv_packet_DAQ).data[4];
                cout << "\n Previous state \n" << *recv_packet_DAQ;
        	    
        	    double position = previous_state(2);
        
                u = PID_control -> getControl(previous_state, reference_position, position );
        
                /* ** Clinet send control data ** */
                
                /*send_packet.CLIENT_HEADER  = '0';
                send_packet.control_cmd[0] = u(0);
                send_packet.control_cmd[1] = u(1);
                send_packet.control_cmd[2] = u(2);*/
                //control cmd send
                send_packet.CLIENT_HEADER = '0';
                send_packet.control_cmd[0] = u(0);
                send_packet.control_cmd[1] = u(1);
                
                buffer_send = (char*)&send_packet;
                //cout << "\n after buffer load and before client send :" << buffer_send;
                struct udppacket_control *asp_control = &send_packet;
                std::cout << "\n  client message send is unsigned int control: " << *asp_control << std::endl;
                PID_control -> client_send(buffer_send, sizeof(send_packet));
                	    
        	    break;
    	    }
    		    
    		    
    		    
            case 'b' :
    	    {
    		    recv_packet_COUNTER = (udppacket_COUNTER *)recv_buffer;
    		    cout << "\n recv_packet_COUNTER \n" << *recv_packet_COUNTER;    
        	    // countersreset send
        	    send_packet_countersreset.CLIENT_HEADER = '1';
                send_packet_countersreset.data = true;//u(0);
                buffer_send = (char*)&send_packet_countersreset;
                struct udppacket_countersreset *asp_countersreset = &send_packet_countersreset;
                std::cout << "\n  client message send is bool countersreset: " << *asp_countersreset << std::endl;
                PID_control -> client_send(buffer_send, sizeof(send_packet_countersreset));
        	    
        	    break;
            }
            
            case 'c' :
    	    {
    		    recv_packet_error = (udppacket_error *)recv_buffer;
    		    cout << "\n recv_packet_error \n" << *recv_packet_error;    
        	    // digitaloutputcontrol send
                send_packet_digitaloutputcontrol.CLIENT_HEADER = '2';
                send_packet_digitaloutputcontrol.data = false;//u(0);
                
                buffer_send = (char*)&send_packet_digitaloutputcontrol;
                //cout << "\n after buffer load and before client send :" << buffer_send;
                struct udppacket_digitaloutputcontrol *asp_digitaloutputcontrol = &send_packet_digitaloutputcontrol;
                std::cout << "\n  client message send is bool digitaloutputcontrol: " << *asp_digitaloutputcontrol << std::endl;
                PID_control -> client_send(buffer_send, sizeof(send_packet_digitaloutputcontrol));
                   
        	    break;
            }
    		    
    		    
    		    
    		    
    		   
    	}
        

       
        /* Data storage */
        whileloop_counter++;
        reference_traj.row(whileloop_counter) << 0, 0, reference_position, 0;
        state_traj.row(whileloop_counter) = previous_state;
        control_traj.row(whileloop_counter) = u;
        /*Data storage ends*/

                
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
           
}
             
