/* Simple demo showing how to communicate with Net F/T using C language. */

#ifdef _WIN32
	#include <winsock2.h>
	#include <ws2tcpip.h>
	#pragma comment(lib, "Ws2_32.lib")
    #include <windows.h>

    #define strcasecmp _stricmp
    #define strncasecmp _strnicmp
#else
	#include <arpa/inet.h>
	#include <sys/socket.h>
    #include <sys/time.h>
	#include <netdb.h>
    #include <unistd.h>
#endif

#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PORT 49152 // Port the Net F/T always uses
#define COMMAND_STREAM 2 //
#define COMMAND_BATCH 3 // Command code 2 starts streaming, 3 for buffered 
#define COMMAND_STOP 0  // stop streaming;
#define NUM_SAMPLES 40 // everytime send 40 samples (the same with netbox control ip)

#define DISP_MAX_CNT    100	// display force data once a second
#define AVG_MAX_CNT     5  // average 5 samples

#include  "/home/vr/RTMA/include/RTMA.h" //"Dragonfly.h"
#include "/home/vr/rg2/include/RTMA_config.h" //"Dragonfly_config.h"

#include <iostream> // cg: this block I changed it
using std::cerr; 
using std::endl;
#include <fstream> // for writing file
using std::ofstream;
#include <cstdlib> // for exit function
#include <time.h>

/* Typedefs used so integer sizes are more explicit */
typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef unsigned char byte;z
typedef struct response_struct {
	uint32 rdt_sequence;
	uint32 ft_sequence;
	uint32 status;
	int32 FTData[6];
} RESPONSE;

static RTMA_Module mod;

#ifdef _WINDOWS_C
	//Global counter frequency value- used by GetAbsTime() on Windows
	double win_counter_freq;
#endif

const std::string currentDateTime(){ // for saving file name
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf) , "%Y%m%d%H%M%S", &tstruct);
  return buf;
}

void InitializeAbsTime( void){
#ifdef _WINDOWS_C
	LONGLONG freq;
	QueryPerformanceFrequency( (LARGE_INTEGER*) &freq);
	win_counter_freq = (double) freq;
#endif
}

void wait(unsigned int milliseconds){
#ifdef _WIN32
    Sleep(milliseconds);
#else
    usleep(milliseconds * 1000);
#endif
}


double GetAbsTime( void){
//WIN: returns a seconds timestamp from a system counter

#ifdef _UNIX_C
    struct timeval tim;
    if ( gettimeofday(&tim, NULL)  == 0 )
    {
        double t = tim.tv_sec + (tim.tv_usec/1000000.0);
        return t;
    }else{
        return 0.0;
    }
#else
    LONGLONG current_time;
    QueryPerformanceCounter( (LARGE_INTEGER*) &current_time);
    return (double) current_time / win_counter_freq;
#endif
}

double elapsed = 0.0;
double elapsed_cnt = 0.0;
double tot_elapsed = 0.0;
double max_elapsed = 0.0;
double max_force[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
bool sendData = false;

int main ( int argc, char ** argv ) {
    double t00 = GetAbsTime();
    
#ifdef _WIN32
	SOCKET socketHandle;		/* Handle to UDP socket used to communicate with Net F/T. */
	WSADATA wsaData;
    WORD wVersionRequested;
#else
	int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */
#endif
	struct sockaddr_in addr;	/* Address of Net F/T. */
	struct hostent *he;			/* Host entry for Net F/T. */
	byte request[8];			/* The request data sent to the Net F/T. */
	RESPONSE resp;				/* The structured response received from the Net F/T. */
	byte response[36*NUM_SAMPLES];			/* The raw response data received from the Net F/T. */ 
                                      // as receieve NUM_SAMPLES datapoints per batch, multiply by length per datapoints 
	int i;						/* Generic loop/array index. */
	int err;					/* Error status of operations. */
    int keep_going = 1;
    CMessage inMsg;

	if ( argc < 2 ){
		fprintf( stderr, "Usage: %s config mm_ip\n", argv[0] );
		return -1;
	}

    InitializeAbsTime();

#ifdef _WIN32
	wVersionRequested = MAKEWORD(2, 2);
    WSAStartup(wVersionRequested, &wsaData);
#endif

	// Calculate number of samples, command code, and open socket here.
	socketHandle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socketHandle == -1) {
		fprintf(stderr, "Socket could not be opened.\n");
		exit(1);
	}

	*(uint16*)&request[0] = htons(0x1234); // standard header.
	*(uint16*)&request[2] = htons(COMMAND_BATCH); // per table 9.1 in Net F/T user manual.
	*(uint32*)&request[4] = htonl(0); // see section 9.1 in Net F/T user manual.

	// Sending the request.
	he = gethostbyname("192.168.2.45"); //(argv[1]);
	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);

	err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
	if (err == -1) {
		exit(2);
	}

    mod.InitVariables(MID_NETBOX_MODULE, 0);

    if( argc > 2){
        char *mm_ip = NULL;
        mm_ip = argv[2];
        printf("connecting to dragonfly at %s\n", mm_ip);
        mod.ConnectToMMM(mm_ip);
    }
    else{
        printf("connecting to dragonfly\n");
        mod.ConnectToMMM("192.168.2.48:7112");
    }

    mod.Subscribe(MT_EXIT);
    mod.Subscribe(MT_PING);
    mod.Subscribe(MT_SAMPLE_GENERATED);
    mod.Subscribe(MT_MOVE_HOME);
    mod.Subscribe( MT_TASK_STATE_CONFIG );

    fprintf( stderr, "Connected to Dragonfly\n");
    
    //declear message and data
    CMessage ForceSensorDataMsg( MT_FORCE_SENSOR_DATA); //message
    ForceSensorDataMsg.AllocateData( sizeof(MDF_FORCE_SENSOR_DATA));
    MDF_FORCE_SENSOR_DATA *force_data[NUM_SAMPLES];     //data
    for(int i=0; i<NUM_SAMPLES; i++){
      force_data[i] = (MDF_FORCE_SENSOR_DATA*) malloc(sizeof(MDF_FORCE_SENSOR_DATA));
    }
    force_data[NUM_SAMPLES-1] = {(MDF_FORCE_SENSOR_DATA *) ForceSensorDataMsg.GetDataPointer()}; // change pointer to struct
    
    CMessage RawForceSensorDataMsg( MT_RAW_FORCE_SENSOR_DATA); //message
    RawForceSensorDataMsg.AllocateData( sizeof(MDF_RAW_FORCE_SENSOR_DATA));
    MDF_RAW_FORCE_SENSOR_DATA *raw_force_data[NUM_SAMPLES];    //data
    for(int i=0; i<NUM_SAMPLES; i++){
      raw_force_data[i] = (MDF_RAW_FORCE_SENSOR_DATA*) malloc(sizeof(MDF_RAW_FORCE_SENSOR_DATA));
    }
    raw_force_data[NUM_SAMPLES-1] = {(MDF_RAW_FORCE_SENSOR_DATA *) RawForceSensorDataMsg.GetDataPointer()}; // change pointer to struct

    int disp_cnt = DISP_MAX_CNT;
    MDF_SAMPLE_GENERATED sample_gen;
    MDF_TASK_STATE_CONFIG tsc;
    double TempAvgForce[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double AvgForce[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int AvgCnt = 0;
    
    ofstream outdata; // written file
    std::string fname;
    fname = "/home/vr/rg2/data/forceSensor/" + currentDateTime() + ".dat";  // ^`___`^ not to hard code, change a directory by message!
                                                                            // how to limit the file size? ask! Em
    const char *cstr_fname = fname.c_str();
    outdata.open(cstr_fname);
    outdata<<"RDT,FT,status,Fx,Fy,Fz,Tx,Ty,Tz,Fx0,Fy0,Fz0,Tx0,Ty0,Tz0,elapse"<<endl;
    send( socketHandle, (const char *)request, 8, 0 ); 	//*** get a bunch of data, and read manual of netbox.
    while(keep_going){
      bool got_msg = mod.ReadMessage( &inMsg, 0);
      
      // collect data anyway, then decide whether sendRTMA/else
      double t0 = GetAbsTime();
      //send( socketHandle, (const char *)request, 8, 0 ); 	//*** get a bunch of data, and read manual of netbox.
      double t01 = GetAbsTime();
      recv( socketHandle, (char *)response, 36*NUM_SAMPLES, 0 );			//*** get information from sensor
      double t1 = GetAbsTime();
      // raw_force_data
      for (int i=0; i<NUM_SAMPLES; i++){
        raw_force_data[i]->sample_header = sample_gen.sample_header;
        raw_force_data[i]->rdt_sequence = ntohl(*(uint32*)&response[i*36+0]);
        raw_force_data[i]->ft_sequence = ntohl(*(uint32*)&response[i*36+4]);
        raw_force_data[i]->status = ntohl(*(uint32*)&response[i*36+8]);
        for( int j = 0; j < 6; j++ ) {
					int32 data;
					data = ntohl(*(int32*)&response[i * 36 + 12 + j*4]); // 36byte each data point, 12 byte offset
					raw_force_data[i]->data[j] = (double)(((double) data) / ((double) 1000000)); 
					}
      }
      double t2 = GetAbsTime();
      double rotF[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      
      for (int i = 0; i < NUM_SAMPLES; i++) { // get force data, send force data, and write in file
        for (int j=0; j<6; j++) rotF[j] = 0.0;
        if (AvgCnt < AVG_MAX_CNT){
          for( int j = 0; j < 6; j++ ) {
            TempAvgForce[j] = TempAvgForce[j] + raw_force_data[i]->data[j];
          }
            AvgCnt = AvgCnt + 1;
        }
        if (AvgCnt == AVG_MAX_CNT){
          for( int j = 0; j < 6; j++ ) {
            AvgForce[j] = TempAvgForce[j] / AVG_MAX_CNT;
          }
          AvgCnt = AvgCnt + 1; // this will stop the averaging until next time
        }
        // force_data
				force_data[i]->sample_header = sample_gen.sample_header;
				force_data[i]->rdt_sequence = raw_force_data[i]->rdt_sequence;
				force_data[i]->ft_sequence = raw_force_data[i]->ft_sequence;
				force_data[i]->status = raw_force_data[i]->status;
        
				for(int j = 0; j < 6; j++ ) {
					force_data[i]->data[j] = raw_force_data[i]->data[j] - AvgForce[j];
					force_data[i]->offset[j] = AvgForce[j];
				}
        
        int j, k;

/*        double rotMat[6][6] = {
          //{0.37695137, 0.92623305, 0.0, 0.0, 0.0, 0.0},
          //{-0.92623305, 0.37695137, 0.0, 0.0, 0.0, 0.0},
          {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
          {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
          {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
          {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
          {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}
          };

        for( j=0;j<6;j++) {
          for( k=0;k<6;k++) {
            rotF[j] += force_data[i]->data[k]*rotMat[k][j];
					}
				}
*/

				for( j=0;j<6;j++) {
					// force_data[i]->data[j] = rotF[j];
					if ( fabs(force_data[i]->data[j]) > max_force[j])
						max_force[j] = force_data[i]->data[j];
				}
        }
        double t3 = GetAbsTime();
        // writing on disk
      for( int i=0; i<NUM_SAMPLES; i++){
          outdata<<ntohl(*(uint32*)&response[36*i+0])<<","; //RDT seq
          outdata<<ntohl(*(uint32*)&response[36*i+4])<<","; //F/T seq
          outdata<<ntohl(*(uint32*)&response[36*i+8])<<","; //status
          for (int j=0; j<6; j++) outdata<<force_data[i]->data[j]<<",";
          for (int j=0; j<6; j++) outdata<<force_data[i]->offset[j]<<",";
          outdata<<endl; //elapse
        }
        double t4 = GetAbsTime();
      if (inMsg.msg_type == MT_TASK_STATE_CONFIG){
          MDF_TASK_STATE_CONFIG task_state_data;
          inMsg.GetData( &task_state_data);
               switch(task_state_data.id)
      {
        case 1:
          cout << " ST 1, ";
          sendData = false;
          break;
        case 2:
          cout << " ST 2, ";
          break;
        case 3: 
          cout << " ST 3, ";
          sendData = true;
          break;
        case 4: //Move
          cout << " ST 4, ";
          break;
        case 5: // hold
          cout << " ST 5, ";
          break;
        case 6:
          cout << " ST 6, ";
          break;
        case 7:
          cout << " ST 7, " << endl;
          break;
        default:
          break;
      }      


      }
      // Recalibrate the force module while its moving home
			if (inMsg.msg_type == MT_MOVE_HOME){
                inMsg.GetData(&tsc);
                AvgCnt = 0;
                for(i=0; i<6; i++)
                  TempAvgForce[i] = (double) 0.0;
      }
 /*     else if (inMsg.msg_type == MT_SESSION_INFO){ // ^`___`^ for saving in a regular filename
        // data_root_folder
        // subject
        // session_no
*/
			else if (inMsg.msg_type == MT_PING){
        cout << "ping sent" << endl;
				char MODULE_NAME[] = "NetboxModule";
				MDF_PING *pg = (MDF_PING *) inMsg.GetDataPointer();
				if ( (strcasecmp(pg->module_name, MODULE_NAME) == 0) ||
				   (strcasecmp(pg->module_name, "*") == 0) ||
				   (inMsg.dest_mod_id == mod.GetModuleID()) ){
					CMessage PingAckMessage( MT_PING_ACK);
					PingAckMessage.AllocateData( sizeof(MDF_PING_ACK));
					MDF_PING_ACK *pa = (MDF_PING_ACK *) PingAckMessage.GetDataPointer();

					memset(pa,0,sizeof(MDF_PING_ACK));
					for (int i = 0; i < strlen(MODULE_NAME); i++){
						pa->module_name[i] = MODULE_NAME[i];
					}

          cout << "ping ack" << endl;
					mod.SendMessage( &PingAckMessage);
				}
			}
                
      else if (inMsg.msg_type == MT_SAMPLE_GENERATED && sendData){
				double t_2 = GetAbsTime();
        
        inMsg.GetData(&sample_gen);
        
        mod.SendMessage(&RawForceSensorDataMsg); //why send data twice??
        mod.SendMessage(&ForceSensorDataMsg);

        // write force data in file, 
        // sequentially: rdt_sequence, ft_sequence, status, , data[6]

				double t_3 = GetAbsTime();

        elapsed = t_3-t_2;
        elapsed_cnt++;
        tot_elapsed += elapsed;
        if (elapsed > max_elapsed)
            max_elapsed = elapsed;

        disp_cnt++;
        

        
        if (disp_cnt >= DISP_MAX_CNT){

            cout << "\n\nelapsed    : " << 1000*elapsed << " msec" << endl;
            cout << "elapsed_snd   : " << 1000*(t01-t0) << "msec" <<endl;
            cout << "elapsed_rcv   : " << 1000*(t1-t01) << "msec" <<endl;
            cout << "elapsed_raw   : " << 1000*(t2-t1) << "msec" <<endl;
            cout << "elapsed_foc   : " << 1000*(t3-t2) << "msec" <<endl;
            cout << "elapsed_wit   : " << 1000*(t4-t3) << "msec" <<endl;
            cout << "avg elapsed: " << 1000*tot_elapsed/elapsed_cnt << " msec" << endl;
            cout << "max elapsed: " << 1000*max_elapsed << " msec" << endl;
            printf("\n");

            // Output the response data
/*
            printf( "rdt: %08d   ", force_data[NUM_SAMPLES-1]->rdt_sequence );
            printf( "ft : %08d   ", force_data[NUM_SAMPLES-1]->ft_sequence );
            printf( "sta: 0x%08x   ", force_data[NUM_SAMPLES-1]->status );
            printf("\n");

            printf( "RAW: ");

              for (i =0;i < 6;i++) {
                printf("%.2lf  ", raw_force_data[NUM_SAMPLES-1]->data[i]);
              }
              printf("\n");

            printf( "OFS: ");

            for (i =0;i < 6;i++) {
                printf("%.2lf  ", AvgForce[i]);
            }
            printf("\n");

            printf( "ADJ: ");
            for (i =0;i < 6;i++) {
              printf("%.2lf  ", force_data[NUM_SAMPLES-1]->data[i]);
            }
            printf("\n\n");

            printf( "ROT: ");
            for (i =0;i < 6;i++) {
              printf( "%.2lf  ", rotF[i]);
            }
            printf("\n\n");

            printf( "Max Force: ");
            for (i =0;i < 6;i++) {
                printf("%.2lf  ", max_force[i]);
            }
            printf("\n\n");

            disp_cnt = 0;
            */
        }
      }
      
			else if (inMsg.msg_type == MT_EXIT){
				if ((inMsg.dest_mod_id == 0) || (inMsg.dest_mod_id == mod.GetModuleID()))
				{
					printf("got exit!\n");
					mod.SendSignal(MT_EXIT_ACK);
					mod.DisconnectFromMMM();
					keep_going = 0;
					break;
				}
			}
    }
  outdata.close(); //close file

// close streamming
*(uint16*)&request[2] = htons(COMMAND_STOP); 
send( socketHandle, (const char *)request, 8, 0 ); 	

	/* Close socket */
#ifdef _WIN32
	closesocket(socketHandle);
#else
	close(socketHandle);
#endif
	return 0;
}
