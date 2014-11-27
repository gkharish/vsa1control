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
                        
            cout << "\n" << "pid coeffs are: " << coefficient;
            return control;
        }
        
        /* Setting coeeffecient of PID controller  */
        void control_model::setpidcoeff(int p, int i, int d)
        {
            _p = p;
            _i = i;
            _d= d;
            cout << "\n" << "pid coeff is set";
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
        
        
        
        
        
        
        /* Real Time function  */
        RT_TASK principal_task;
        MatrixXd outputprint;
        
        /*  Variables used in data storage */
        MatrixXd reference_traj, state_traj, control_traj;
            
            
            
        /* *****     PRINCIPAL FUNCTION       ***** */
        void principal_function(void *argv)
        {
    
            
            cout << "\n INSIDE Principal_function";
            
            /* variables used in the principal program */
            int whileloop_counter = 0, error_counter = 0, loop = 0;
            int timeofsimulation_s = 30; /* time in seconds*/
            int FLAG = 1;
            int nsig;
        
        /*Variables of system dynamics state space*/
            VectorXd initial_state(4); 
            initial_state << 0,
            0,
            0,
            0;
        
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
            state_traj.resize (timeofsimulation_s*numsamples_s, initial_state.size () );
            state_traj.row(0) = initial_state;
            
            reference_traj.resize(timeofsimulation_s*numsamples_s, ref_init.size());
            reference_traj.row(0) = ref_init;
            
            control_traj.resize(timeofsimulation_s*numsamples_s, u.size());
            control_traj.row(0) =  u;
            
            
            //rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD));
            
            now = rt_timer_read();
            time_start_loop  = round(now/1.0e9);
            cout << "Start of while loop:" << time_start_loop;
            
            ostringstream u_char;
            /*u_char1 << 2.31; u_char2 << 3.31; u_char3 << 0.001;
            strcpy(buf, u_char1.str().c_str());
            strcat(buf, u_char2.str().c_str());
            strcat(buf, u_char3.str().c_str());*/
            //cout << buf;
            
            //  *** sending controls**** // 
            int control_len, control_size, state_len, states_size;
            control_size= u.size();
            states_size = previous_state.size();
            
            /**/
            cout << "\n Sending request to server .....";
            strcpy(buf,"Start");
            PID_control -> client_send(buf);
            cout << "\n Client running .....";
            
            /*for(control_len = 0; control_len <= control_size-1; control_len++)
            {
               u_char << u(control_len); 
               strcpy(buf, u_char.str().c_str());
               PID_control -> client_send(buf);
               //flush buf
            }*/
            
            
            while(FLAG)
            {
                rt_task_wait_period(NULL);
                
                now = rt_timer_read();
                present_time  = round(now/1.0e9);
                t = present_time - time_start_loop;
                double reference_position = reference_generator(ref_pos);
                cout << "inside client while ";
                
                for(state_len = 0; state_len <= states_size -1; state_len++)
                {
                    //memset(buf, 0, sizeof(buf));
                    
                    PID_control -> client_recv(buf);
                    //double position = atof(buf);
                    previous_state(state_len) = atof(buf);
                    cout << "inside client recv for loop";
                    //flush buf
                    
                }
                
                
                double position = previous_state(2);     //atof(buf);//*( ( double*)buffer_server_recv);
                //previous_state(2) = position;
                u = PID_control -> getControl(previous_state, reference_position, position );
                //cout << "\ngetcontrol:" << u;
                
                //ostringstream u_char;
                //u_char << u(0);
                //strcpy(buf, u_char.str().c_str());
                 
                //PID_control -> client_send(buf);
                
                for(control_len = 0; control_len <= control_size-1; control_len++)
                {
                   
                   
                   u_char << u(control_len); 
                   strcpy(buf, u_char.str().c_str());
                   strcpy(buf,"%");
                   //PID_control -> client_send(buf);
                   //flush buf
                  
                }
                PID_control -> client_send(buf);
                /* Data storage */
               /* whileloop_counter++;
                reference_traj.col(whileloop_counter) << 0,0, reference_position,0;
                state_traj.row(whileloop_counter)(2) = position;
                control_traj.row(whileloop_counter) = u;*/
                /*Data storage ends*/
                
                /*state_traj = trajectorystore(previous_state, whileloop_counter);
                control_traj = trajectorystore(u, whileloop_counter); 
                reference_traj = trajectorystore(ref_init, whileloop_counter);*/
                
                whileloop_counter++;
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
            
            ofstream statetrajdata, referencetrajdata, controltrajdata;;
            signal(SIGTERM, catch_signal);
	        signal(SIGINT, catch_signal);
	        char* a;
	        mlockall(MCL_CURRENT|MCL_FUTURE);
	        
	        cout << "INSIDE MAIN" << endl;

        	n = rt_task_create(&principal_task, "principal_function", 0, 99, 0);
            //principal_function();
            
            //reference_generator(30.0);
        	
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
            //cin.ignore().get();
            cout << "END of Pause" << endl;
        	//result print
        
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
             
/*int main(void)
{
    //char buffer_client[BUFLEN];
    //server = "127.0.0.1";
    char buf[BUFLEN];
    //char buffer_server;
    control_model *PID_control = new control_model();
    PID_control -> client_start();
    
    /*create a packet to be sent*/
    
/*    for (int i=0; i < MSGS; i++) 
    {
      //std::cout << "\n Sending packet" <<  i <<"to port "  <<  SERVICE_PORT << "\n";
		sprintf(buf, "This is packet %d", i);
    
        PID_control -> client_send(buf);
        
        PID_control -> client_recv(buf);
    }
}*/
