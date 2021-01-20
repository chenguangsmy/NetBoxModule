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
#include <fstream> // for writing file
#include <cstdlib> // for exit function
#include <time.h>
#include <pthread.h>
#include "Netboxrec.h"
#include "NetboxRTMA.h"

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
string fname;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
FLAGS flag;
RESPONSE forcedat; 

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

void respondRTMA(NetftRTMA *netRTMA)
{
  printf("Start respondRTMA function!");
  while (1)
  {
    //receieve Msg
    netRTMA->receive();
    //respond Msg
    printf("G: finish receive, before lock 00 \n");
    printf("G: finish receive, before lock 01 \n");
    pthread_mutex_lock(&mutex);
    printf("G: finish receive, lock, before respond\n");
    flag = netRTMA->respond(&forcedat, flag);
    fname = netRTMA->fname;
    pthread_mutex_unlock(&mutex);
    printf("end of netRTMA->respond\n");
    if (keep_going == false)
    {
      printf("Keep_going is %d\n", keep_going);
      printf("enter break point. \n");
      break;
    }
    printf("G: end of one respondRTMA loop\n");
  }
  printf("G: Finish respondRTMA function!");
}

void *processNetrec(void *arg)
{ // how to deal with argument proboem?
  Netboxrec *netrec = (Netboxrec *)arg;
  printf("Enter thread netrec! \n");
  while (1)
  {
    printf("G: start processNetrec loop! \n");
    // mutex_lock flag
    pthread_mutex_lock(&mutex);
    printf("G: start processNetrec loop, lock finished! \n");
    netrec->getforceData();
    printf("G: start processNetrec loop, lock finished01! \n");
    forcedat = netrec->getforceData();
    if (flag.stream)
    {
      printf("G: start streaming! ");
      netrec->recvstream();
      netrec->writeFile();
    }
    if (flag.UpdateAvg)
    {
		printf("G: flag.Updateavg! \n");
      flag.UpdateAvg = false;
    }
    if (flag.FileInit)
    {
		printf("G: flag.FileInit! \n");
      netrec->fileInit(fname.c_str()); // init file here
      flag.FileInit = false;
    }
    if (flag.SendRequest)
    {
		printf("G: flag.SendRequest! \n");
      flag.SendRequest = false;
    }
    if (flag.stopStream)
    {
		printf("G: flag.Stopstream! \n");
      flag.stream = false;
      flag.stopStream = false;
    }
    if (flag.CloseFile)
    {
		printf("G: flag.CloseFile! \n");
      flag.CloseFile = false;
    }
    //mutex unlock flag;
    pthread_mutex_unlock(&mutex);
    printf("G: end processNetrec loop! \n");
  }
  printf("G: Finish thread netrec! \n");
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
  respondRTMA(&netRTMA);
  pthread_join(netrec_thread, NULL);
  netrec.socketClose();
  return 0;
}
