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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "/home/vr/RTMA/include/RTMA.h"
#include "/home/vr/rg2/include/RTMA_config.h"
#include <iostream> // cg: this block I changed it
#include <fstream>  // for writing file
#include <cstdlib>  // for exit function
#include <time.h>
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
bool keep_going = true;

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
  int argc;
  char **argv;
  //static RTMA_Module mod;
  RTMA_Module mod;
  CMessage ForceSensorDataMsg;              //message
  MDF_FORCE_SENSOR_DATA force_data;         //data
  CMessage RawForceSensorDataMsg;           //message
  MDF_RAW_FORCE_SENSOR_DATA raw_force_data; //data
  MDF_SAMPLE_GENERATED sample_gen;
  MDF_TASK_STATE_CONFIG tsc;
  bool got_msg;
  CMessage inMsg;

private:
protected:
  // functions;
public:
  // connectTo();
  void receive()
  {
    got_msg = mod.ReadMessage(&inMsg, 0);
  }
  void respond(Netboxrec *netrec);
  void writeMsg(Netboxrec *netrec)
  {
    i = 0;
    // only package the first message in a set of NUM_SAMPLES
    raw_force_data.sample_header = sample_gen.sample_header;
    raw_force_data.rdt_sequence = netrec->rdt_sequence[i];
    raw_force_data.ft_sequence = netrec->ft_sequence[i];
    raw_force_data.status = netrec->status[i];
    force_data.sample_header = sample_gen.sample_header;
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
    // variables
    got_msg = false;
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

    //allocate size ? how to deal with this? as size = 1?
    /*
  ForceSensorDataMsg.AllocateData(sizeof(MDF_FORCE_SENSOR_DATA));
  for (i = 0; i < NUM_SAMPLES; i++)
  {
    force_data[i] = (MDF_FORCE_SENSOR_DATA *)malloc(sizeof(MDF_FORCE_SENSOR_DATA));
  }
  force_data[NUM_SAMPLES - 1] = {(MDF_FORCE_SENSOR_DATA *)ForceSensorDataMsg.GetDataPointer()}; // change pointer to struct
  RawForceSensorDataMsg.AllocateData(sizeof(MDF_RAW_FORCE_SENSOR_DATA));
  for (i = 0; i < NUM_SAMPLES; i++)
  {
    raw_force_data[i] = (MDF_RAW_FORCE_SENSOR_DATA *)malloc(sizeof(MDF_RAW_FORCE_SENSOR_DATA));
  }
  raw_force_data[NUM_SAMPLES - 1] = {(MDF_RAW_FORCE_SENSOR_DATA *)RawForceSensorDataMsg.GetDataPointer()}; // change pointer to struct
*/
    mod.Subscribe(MT_EXIT);
    mod.Subscribe(MT_PING);
    mod.Subscribe(MT_SAMPLE_GENERATED);
    mod.Subscribe(MT_MOVE_HOME);
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
    netrec->updateAvg();
    inMsg.GetData(&tsc);
  }
  /*   else if (inMsg.msg_type == MT_SESSION_INFO)
    { // ^`___`^ for saving in a regular filename
      // data_root_folder
      // subject
      // session_no
    }*/
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
      keep_going = 0;
      //break;
    }
  }
}

int main(int argc, char **argv)
{
  NetftRTMA netRTMA(argc, argv);
  Netboxrec netrec;
  t0 = GetAbsTime();
  netrec.socketInit();
  InitializeAbsTime();
  netrec.sendrequest();

  int disp_cnt = DISP_MAX_CNT;
  netrec.fileInit();
  netrec.sendrequest();
  while (keep_going)
  {
    // consider RTMA as another thread? Thus there would be less delay...
    netRTMA.receive();
    t0 = GetAbsTime();
    netrec.recvstream();
    t1 = GetAbsTime();
    netRTMA.writeMsg(&netrec);
    t3 = GetAbsTime();
    // writing on disk
    netrec.writeFile();
    t4 = GetAbsTime();
    // Recalibrate the force module while its moving home
    netRTMA.respond(&netrec);
    t3 = GetAbsTime();

    elapsed = t3 - t2;
    elapsed_cnt++;
    tot_elapsed += elapsed;
    if (elapsed > max_elapsed)
      max_elapsed = elapsed;
    disp_cnt++;

    if (disp_cnt >= DISP_MAX_CNT)
    {
      cout << "\n\nelapsed    : " << 1000 * elapsed << " msec" << endl;
      cout << "elapsed_raw   : " << 1000 * (t2 - t1) << "msec" << endl;
      cout << "elapsed_foc   : " << 1000 * (t3 - t2) << "msec" << endl;
      cout << "elapsed_wit   : " << 1000 * (t4 - t3) << "msec" << endl;
      cout << "avg elapsed: " << 1000 * tot_elapsed / elapsed_cnt << " msec" << endl;
      cout << "max elapsed: " << 1000 * max_elapsed << " msec" << endl;
      printf("\n");

      // Output the ft_response data
      /*
      printf("rdt: %08d   ", force_data[NUM_SAMPLES - 1]->rdt_sequence);
      printf("ft : %08d   ", force_data[NUM_SAMPLES - 1]->ft_sequence);
      printf("sta: 0x%08x   ", force_data[NUM_SAMPLES - 1]->status);
      printf("\n");

      printf("RAW: ");

      for (i = 0; i < 6; i++)
      {
        printf("%.2lf  ", raw_force_data[NUM_SAMPLES - 1]->data[i]);
      }
      printf("\n");
      printf("ADJ: ");
      for (i = 0; i < 6; i++)
      {
        printf("%.2lf  ", force_data[NUM_SAMPLES - 1]->data[i]);
      }
      printf("\n\n");
*/
      disp_cnt = 0;
    }
  }
  netrec.closeFile();
  netrec.stopStream();
  netrec.socketClose();
  return 0;
}
