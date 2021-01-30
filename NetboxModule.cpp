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
bool keep_going = true;
bool fwriting = false;
bool fileopen = false;
string fname;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
FLAGS flag;
RESPONSE *forcedat; 

void InitializeAbsTime(void)
{
#ifdef _WINDOWS_C
  LONGLONG freq;
  QueryPerformanceFrequency((LARGE_INTEGER *)&freq);
  win_counter_freq = (double)freq;
#endif
}

void *respondRTMA(NetftRTMA *netRTMA, Netboxrec *netrec)
{
  while (netRTMA->flag_exit == false)
  {
    //receieve Msg
    netRTMA->receive();
    //pthread_mutex_lock(&mutex);
    // respond with certain netrec fuctions, including:
    //    sendrequestStart();
    //    sendrequestStop();
    //    filInit(fname);   //init a file
    //    setStreamStart(); //set a flag for stream loop as true;
    //    setStreamStop()
    //    fileClose();      // close the wrote file
    netRTMA->respondr(forcedat, netrec);
    // Cannot guarentee if the same data would send twice, as these two is not in the same thread. 
    // Or need to add additional flags to compensate.  
    //pthread_mutex_unlock(&mutex);
    //printf("end of netRTMA->respond\n");
    if (netRTMA->flag_exit == false)
    {
      keep_going = true;
    }
    else
    {
      keep_going = false;
      break;
    }
    //printf("RTMA: end of one respondRTMA loop\n");
  }
  printf("RTMA: Finish respondRTMA function!");
}

void *processNetrec(void *arg)
{ 
  Netboxrec *netrec = (Netboxrec *)arg;
  while (keep_going) 
  {
    if (netrec->getStreamStatus()) // if true, start stream; 
    {
	  //printf("Streaming...\n");
	  flag.stream = false;
	  flag.stream = true;
      netrec->recvStream();
      netrec->writeFile();
      pthread_mutex_lock(&mutex);
      netrec->getforceData(forcedat); //Debug: check if the forcedat have been copied correctly!
                                      // both main thread and netrec visit this forcedat, Lock here?
      pthread_mutex_unlock(&mutex);
    }
    if (!keep_going)
    { 
      break;
    }
    
  }
  printf("NREC: Finish thread netrec! \n");
}

int main(int argc, char **argv)
{
  forcedat = (RESPONSE*) malloc(sizeof(RESPONSE));
  NetftRTMA netRTMA(argc, argv);
  Netboxrec netrec;
  netrec.socketInit();
  InitializeAbsTime();
  int disp_cnt = DISP_MAX_CNT;
  pthread_t netrec_thread;
  pthread_create(&netrec_thread, NULL, processNetrec, (void*) &netrec);
  respondRTMA(&netRTMA, &netrec);
  pthread_join(netrec_thread, NULL);
  netrec.socketClose();
  return 0;
}
