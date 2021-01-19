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
#ifdef _WINDOWS_C
//Global counter frequency value- used by GetAbsTime() on Windows
double win_counter_freq;
#endif

#define PORT 49152       // Port the Net F/T always uses
#define COMMAND_STREAM 2 //
#define COMMAND_BATCH 3  // Command code 2 starts streaming, 3 for buffered
#define COMMAND_STOP 0   // stop streaming;
#define RESP_SIZE 36     // the size (byte) of response_struct
#define NUM_SAMPLES 40   // everytime send 40 samples (the same with netbox control ip)
#define DISP_MAX_CNT 100 // display force data once a second
#define AVG_MAX_CNT 5    // average 5 samples

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "/home/vr/RTMA/include/RTMA.h"
#include "/home/vr/rg2/include/RTMA_config.h"
#include <iostream> // cg: this block I changed it
#include <string>
#include <fstream>  // for writing file
#include <cstdlib>  // for exit function
#include <time.h>
#include <pthread.h>
#include "Netboxrec.h"

using std::cerr;
using std::endl;
using std::ofstream;

/* Typedefs used so integer sizes are more explicit */
typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef unsigned char byte;

/* global variables */

double elapsed = 0.0;
double elapsed_cnt = 0.0;
double tot_elapsed = 0.0;
double max_elapsed = 0.0;
double t0, t1, t2, t3, t4;
int i, j, k; /* Generic loop/array index. */
bool keep_going = false;
bool fwriting = false;

void InitializeAbsTime(void)
{
#ifdef _WINDOWS_C
  LONGLONG freq;
  QueryPerformanceFrequency((LARGE_INTEGER *)&freq);
  win_counter_freq = (double)freq;
#endif
}
double GetAbsTime(void)
{
  //WIN: returns a seconds timestamp from a system counter

#ifdef _UNIX_C
  struct timeval tim;
  if (gettimeofday(&tim, NULL) == 0)
  {
    double t = tim.tv_sec + (tim.tv_usec / 1000000.0);
    return t;
  }
  else
  {
    return 0.0;
  }
#else
  LONGLONG current_time;
  QueryPerformanceCounter((LARGE_INTEGER *)&current_time);
  return (double)current_time / win_counter_freq;
#endif
}

class NetftRTMA
{
  // variables;
public:
private:
  //static RTMA_Module mod;
  RTMA_Module mod;
  CMessage ForceSensorDataMsg;              //message
  MDF_FORCE_SENSOR_DATA force_data;         //data
  CMessage RawForceSensorDataMsg;           //message
  MDF_RAW_FORCE_SENSOR_DATA raw_force_data; //data
  MDF_SAMPLE_GENERATED sample_gen;
  MDF_TASK_STATE_CONFIG tsc;
  MDF_SESSION_CONFIG ssconfig;
  MDF_XM_START_SESSION stsession;
  bool got_msg;
  char data_dir[MAX_DATA_DIR_LEN];
  char subject_name[TAG_LENGTH];
  int  session_num;
  string file_name;       //seperation file name
  string file_dir;
  bool flag_sconfig;
  bool flag_xmconfig;
  CMessage inMsg;

protected:
  int argc;
  char **argv;
  // functions;
public:
  // connectTo();
  void receive()
  {
    got_msg = mod.ReadMessage(&inMsg, 0);
  }
  void respond(Netboxrec *netrec);
  void updateMsg(Netboxrec *netrec)
  {
    i = 0;
    // only package the first message in a set of NUM_SAMPLES
    raw_force_data.sample_header = sample_gen.sample_header;
    force_data.sample_header = sample_gen.sample_header;
    // set these variables private and visit them using function
    raw_force_data.rdt_sequence = netrec->rdt_sequence[i];
    raw_force_data.ft_sequence = netrec->ft_sequence[i];
    raw_force_data.status = netrec->status[i];
    force_data.rdt_sequence = netrec->rdt_sequence[i];
    force_data.ft_sequence = netrec->ft_sequence[i];
    force_data.status = netrec->status[i];
    for (j = 0; j < 6; j++)
    {
      raw_force_data.data[j] = netrec->raw_force[i * 6 + j];
      force_data.data[j] = netrec->force[i * 6 + j];
      force_data.offset[j] = netrec->AvgForce[j];
    }
  }

  //
  //private:
  //protected:
  // construction;
public:
  NetftRTMA(int argc, char **argv) : argc(argc), argv(argv)

  {
    printf("entered NetftRTMA construction!\n");
    // variables
    got_msg = false;
    flag_sconfig = false;
    flag_xmconfig = false;
    ForceSensorDataMsg = MT_FORCE_SENSOR_DATA;
    RawForceSensorDataMsg = MT_RAW_FORCE_SENSOR_DATA;
    //functions
    if (argc < 2)
    {
      fprintf(stderr, "Usage: %s config mm_ip\n", argv[0]);
      //  return -1;
    }
    // RTMA initialization
    mod.InitVariables(MID_NETBOX_MODULE, 0);
    if (argc > 2)
    {
      char *mm_ip = NULL;
      mm_ip = argv[2];
      printf("connecting to dragonfly at %s\n", mm_ip);
      mod.ConnectToMMM(mm_ip);
    }
    else
    {
      printf("connecting to dragonfly\n");
      mod.ConnectToMMM("192.168.2.48:7112");
    }

    mod.Subscribe(MT_EXIT);
    mod.Subscribe(MT_PING);
    mod.Subscribe(MT_SAMPLE_GENERATED);
    mod.Subscribe(MT_MOVE_HOME);
    mod.Subscribe(MT_SESSION_CONFIG);
    mod.Subscribe(MT_XM_START_SESSION);

    keep_going = true;
  }

  // deconstruction;
  ~NetftRTMA()
  {
    //clean the space malloced.
  }
};

void NetftRTMA::respond(Netboxrec *netrec)
{
  if (inMsg.msg_type == MT_MOVE_HOME)
  {
    printf("MT_MOVE_HOME: update Avg. \n");
    netrec->updateAvg();
    inMsg.GetData(&tsc);
  }
  else if (inMsg.msg_type == MT_SESSION_CONFIG)
  { 
    inMsg.GetData(&ssconfig);
    strcpy(data_dir, ssconfig.data_dir); //need to convert to char[].c_str()..finished
    file_dir = data_dir;
    flag_sconfig = true;
  }
  else if (inMsg.msg_type == MT_XM_START_SESSION)
  {
    inMsg.GetData(&stsession);
    strcpy(subject_name, stsession.subject_name);
    session_num = stsession.calib_session_id;
    file_name = printf("%s%d", subject_name, session_num);
    flag_xmconfig = true;
  }
  else if (inMsg.msg_type == MT_PING)
  {
    cout << "ping sent" << endl;
    char MODULE_NAME[] = "NetboxModule";
    MDF_PING *pg = (MDF_PING *)inMsg.GetDataPointer();
    if ((strcasecmp(pg->module_name, MODULE_NAME) == 0) ||
        (strcasecmp(pg->module_name, "*") == 0) ||
        (inMsg.dest_mod_id == mod.GetModuleID()))
    {
      CMessage PingAckMessage(MT_PING_ACK);
      PingAckMessage.AllocateData(sizeof(MDF_PING_ACK));
      MDF_PING_ACK *pa = (MDF_PING_ACK *)PingAckMessage.GetDataPointer();

      memset(pa, 0, sizeof(MDF_PING_ACK));
      for (i = 0; i < strlen(MODULE_NAME); i++)
      {
        pa->module_name[i] = MODULE_NAME[i];
      }

      cout << "ping ack" << endl;
      mod.SendMessage(&PingAckMessage);
    }
  }
  else if (inMsg.msg_type == MT_SAMPLE_GENERATED)
  {
    t2 = GetAbsTime();
    inMsg.GetData(&sample_gen);
    // package message here!
    updateMsg(netrec);
    mod.SendMessage(&RawForceSensorDataMsg);
    mod.SendMessage(&ForceSensorDataMsg);
  }
  else if (inMsg.msg_type == MT_EXIT)
  {
    if ((inMsg.dest_mod_id == 0) || (inMsg.dest_mod_id == mod.GetModuleID()))
    {
      printf("got exit!\n");
      mod.SendSignal(MT_EXIT_ACK);
      mod.DisconnectFromMMM();
      keep_going = false;

      netrec->closeFile();
      fwriting = false; 
      netrec->stopStream();

      //break;
    }
  }

  // task conditions logic
  if (flag_sconfig & flag_xmconfig & (~fwriting)){//not writing file, but receieved config, open file
  // concern: if a session has longer (more than a session), will this still work?
    netrec->fileInit(file_dir + '/' + file_name);
    netrec->sendrequest();
    printf("file initialized\n");
    //reset flags
    flag_sconfig = false;
    flag_xmconfig = false;
    fwriting = true;
  }
}

void respondRTMA(NetftRTMA *netRTMA, Netboxrec *netrec)
{
  printf("Start respondRTMA function!");
  while (1)
  {
    //receieve Msg
    netRTMA->receive();
    //respond Msg
    netRTMA->respond(netrec);
    if (keep_going == false){
		printf("Keep_going is %d\n", keep_going);
		printf("enter break point. \n");
		break;
	}
  }
  printf("Finish respondRTMA function!");
}

void *processNetrec(void *arg)
{ // how to deal with argument proboem?
  Netboxrec *netrec = (Netboxrec *)arg;
  printf("Enter thread netrec! \n");
  while (1)
  {
    if (keep_going & fwriting){
    netrec->recvstream();
    netrec->writeFile();
    }
  }
  printf("Finish thread netrec! \n");
}

int main(int argc, char **argv)
{
  NetftRTMA netRTMA(argc, argv);
  Netboxrec netrec;
  netrec.socketInit();
  InitializeAbsTime();

  int disp_cnt = DISP_MAX_CNT;

  pthread_t netrec_thread;
  pthread_create(&netrec_thread, NULL, processNetrec, NULL);

  respondRTMA(&netRTMA, &netrec);
  pthread_join(netrec_thread, NULL);
  netrec.socketClose();
  return 0;
}
