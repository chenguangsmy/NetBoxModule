#pragma once
// defines here
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
#define NUM_SAMPLES 10   // everytime send 40 samples (the same with netbox control ip)
#define DISP_MAX_CNT 100 // display force data once a second
#define AVG_MAX_CNT 5    // average 5 samples

// includes here
#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream> // cg: this block I changed it
#include <fstream>  // for writing file
#include <cstdlib>  // for exit function
#include <time.h>

// using namespace here
using std::cerr;
using std::endl;
using std::ofstream;

// type define here
typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef unsigned char byte;
#ifndef response_struct

typedef struct response_struct
{ // receieved from FT, trans it to NetboxModule.
  uint32 rdt_sequence;
  uint32 ft_sequence;
  uint32 status;
  int32 FTData[6];
  int32 FTAvg[6];
  double time;
} RESPONSE;
#endif
typedef struct response_flags
{ // flags mark how should the Netboxrec function do
  bool UpdateAvg;
  bool CloseFile;
  bool stopStream;
  bool SendRequest;
  bool FileInit;
  bool stream;
} FLAGS;

/* Functions using */
const std::string currentDateTime()
{ // for saving file name
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", &tstruct);
  return buf;
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

class Netboxrec
{
  // variables
public:
#ifdef _WIN32
  SOCKET socketHandle; /* Handle to UDP socket used to communicate with Net F/T. */
  WSADATA wsaData;
  WORD wVersionRequested;
#else
  
#endif
  struct sockaddr_in addr;            /* Address of Net F/T. */
  struct hostent *he;                 /* Host entry for Net F/T. */
  byte ft_request[8];                 /* The request data sent to the Net F/T. */
  RESPONSE ft_resp ;                   /* The structured ft_response received from the Net F/T. */
  FLAGS flag;                         /* Command flag sending from RTMA module */
  byte ft_response[36 * NUM_SAMPLES]; /* The raw ft_response data received from the Net F/T. */
                                      // as receieve NUM_SAMPLES datapoints per batch, multiply by length per datapoints
  int i, j, k;                        /* Generic loop/array index. */
  int err;                            /* Error status of operations. */
  int32 data;                         /* temporary variable in receiving force data*/
  int iter_i; 
  bool keep_going;
  // add a raw force, size 6*MAX_SIZE
  // add a force, size 6*MAX_SIZE
  int AvgCnt = 0;
private:
  bool stream_status; 
  int socketHandle; /* Handle to UDP socket used to communicate with Net F/T. */
  int rdt_sequence[NUM_SAMPLES];
  int ft_sequence[NUM_SAMPLES];
  int status[NUM_SAMPLES];
  double raw_force[6 * NUM_SAMPLES];
  double force[6 * NUM_SAMPLES];
  double time[NUM_SAMPLES];
  double TempAvgForce[6];
  double AvgForce[6];

  ofstream outdata; // written file
  std::string fname;
  const char *cstr_fname;
protected:
  // functions
public:
  int   fileInit(string); // set up the file writing to seperate
  int   fileClose();      // close the file

  int   socketInit();     // set up the socket recording from FT
  int   socketClose();    // close the socket and stop recording
  int   sendRequestStart();    // sending request starting streaming recording
  int   setStreamStart(); // set internal variable stream_status true;
  int   setStreamStop();  // set internal variable stream_status false;
  bool  getStreamStatus(); // return stream_status;
  int   recvStream();     // receive the data from FT
  int   sendRequestStop();    // stop stream of reading
  void  operate();       // while function locates here
  void  display();       // display the recorded data .../tobedone
  int   forceTransform(); // transform data (maybe from torque to force?)
  int   updateAvg();      // get average through a set of data
  void  writeFile();     // write data on .csv file
  int   closeFile();     // close data write stream
  void  showElapse();

  double *getAverageforceData(); // return average force data
  int   getRDT();
  int   getFT();
  int   getstatus();
  void  getforceData(RESPONSE*); 

public:
  Netboxrec()
  {
    printf("enter Netboxrec construction! \n");    //ft_resp = (RESPONSE*) malloc(sizeof(RESPONSE));
    // set initial values of some vars.
    // how to limit the file size? ask! Em
    keep_going = true;
    stream_status = false;
    for (i = 0; i < 6; i++)
    {
      TempAvgForce[i] = 0;
    }
    for (i = 0; i < 6; i++)
    {
      AvgForce[i] = 0;
    }
  }
  ~Netboxrec()
  {
    // .... delete other un-related vars
  }
};
int Netboxrec::setStreamStart()
{
  stream_status = true;
  return 1;
}
int Netboxrec::setStreamStop()
{
  stream_status = false;
  return 1;
}
bool Netboxrec::getStreamStatus()
{
  return stream_status;
}
int Netboxrec::socketInit()
{
  // socket initialization
  socketHandle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  printf("socketHandle is: %d ", socketHandle);
  if (socketHandle == -1)
  {
    fprintf(stderr, "Socket could not be opened.\n");
    exit(1);
    return -1;
  }
  // Sending the ft_request.
  he = gethostbyname("192.168.2.45"); //(argv[1]);
  memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(PORT);

  err = connect(socketHandle, (struct sockaddr *)&addr, sizeof(addr));
  if (err == -1)
  {
    exit(2);
    return -1;
  }
  printf("Socket initialize finished!\n");
  return 1;
}
int Netboxrec::socketClose()
{
  /* Close socket */
#ifdef _WIN32
  closesocket(socketHandle);
#else
  close(socketHandle);
#endif
  return 1;
}
int Netboxrec::fileInit(string fname)
{

  printf("fname is: %s", fname.c_str());
  //outdata = new ofstream;
  outdata.open((const char*) fname.c_str());
  // the head of form should be the same with writing sequence in writeFile()
  outdata << "RDT,FT,status,Fx,Fy,Fz,Tx,Ty,Tz,Fx0,Fy0,Fz0,Tx0,Ty0,Tz0,elapse" << endl;
  printf("try another line \n");
  return 1;
}
int Netboxrec::sendRequestStart()
{
	*(uint16 *)&ft_request[0] = htons(0x1234);        // standard header.
  *(uint16 *)&ft_request[2] = htons(COMMAND_BATCH); // per table 9.1 in Net F/T user manual.
  *(uint32 *)&ft_request[4] = htonl(0);             // see section 9.1 in Net F/T user manual.
  
  send(socketHandle, (const char *)ft_request, 8, 0); //*** get a bunch of data, and read manual of netbox. //-> should move into the response of start session.
}
int Netboxrec::recvStream()
{
  recv(socketHandle, (char *)ft_response, 36 * NUM_SAMPLES, 0); //*** get information from sensor

  for (i = 0; i < NUM_SAMPLES; i++)
  {
    rdt_sequence[i] = ntohl(*(uint32 *)&ft_response[i * 36 + 0]);
    ft_sequence[i] = ntohl(*(uint32 *)&ft_response[i * 36 + 4]);
    status[i] = ntohl(*(uint32 *)&ft_response[i * 36 + 8]);
    for (j = 0; j < 6; j++)
    {
      data = ntohl(*(int32 *)&ft_response[i * 36 + 12 + j * 4]); // 36byte each data point, 12 byte offset
      raw_force[i * 6 + j] = (double)(((double)data) / ((double)1000000));
    }
    for (j = 0; j < 6; j++)
    {
      force[i * 6 + j] = raw_force[i * 6 + j] - AvgForce[j];
    }
    time[i] = GetAbsTime();
  }
  // copy data into resp
  // ...Todo: deal with the elapse! 
  ft_resp.rdt_sequence = rdt_sequence[NUM_SAMPLES-1];
  ft_resp.ft_sequence = ft_sequence[NUM_SAMPLES-1];
  ft_resp.status = status[NUM_SAMPLES-1];
  for (i = 0; i < 6; i++){
    ft_resp.FTData[i] = force[(NUM_SAMPLES-1)*6 + i];
    ft_resp.FTAvg[i] = AvgForce[j];
  }
  ft_resp.time = time[NUM_SAMPLES-1];
}
int Netboxrec::updateAvg()
{
  for (i = 0; i < 6; i++)
  {
    TempAvgForce[i] = 0.0;
  }
  for (i = 0; i < AVG_MAX_CNT; i++)
  {
    for (j = 0; j < 6; j++)
    {
      TempAvgForce[j] = TempAvgForce[j] + raw_force[i * 6 + j];
    }
  }
  for (j = 0; j < 6; j++)
  {
    AvgForce[j] = TempAvgForce[j] / AVG_MAX_CNT;
  }
}
void Netboxrec::writeFile()
{
  // printf("write file! \n");
  for (i = 0; i < NUM_SAMPLES; i++) // according to header sequence
  {
    outdata << rdt_sequence[i] << ","; //RDT seq
    outdata << ft_sequence[i] << ",";  //F/T seq
    outdata << status[i] << ",";       //status
    for (j = 0; j < 6; j++)
    {
      outdata << force[i * 6 + j] << ",";
    }
    for (j = 0; j < 6; j++)
    {
      outdata << AvgForce[j] << ",";
    }
    // ...Todo: add elipse here!
    outdata << time[i]; // receieve time! why the time are always the same???
    outdata << endl; 
  }
}
int Netboxrec::closeFile()
{
	//outdata = new ofstream;
  outdata.close(); //close file
  return 1;
}
int Netboxrec::sendRequestStop()
{
  *(uint16 *)&ft_request[2] = htons(COMMAND_STOP);
  send(socketHandle, (const char *)ft_request, 8, 0);
  return 1;
}
void Netboxrec::getforceData(RESPONSE* forcedat){
	// to *forcedat = *ft_resp;
	memcpy((void*) forcedat, (void*) &ft_resp, sizeof(RESPONSE));
//	return ft_resp;
}
int Netboxrec::forceTransform()
{
  //  double rotF[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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
}
void Netboxrec::showElapse()
{
  //t0 = GetAbsTime();
  //t1 = GetAbsTime();
  //t3 = GetAbsTime();
  // writing on disk
  //t4 = GetAbsTime();
  // Recalibrate the force module while its moving home
  //t3 = GetAbsTime();
  /*
    elapsed = t3 - t2;
    elapsed_cnt++;
    tot_elapsed += elapsed;
    if (elapsed > max_elapsed)
      max_elapsed = elapsed;
*/
  //  disp_cnt++;
  /*  if (disp_cnt >= DISP_MAX_CNT)
    {
      cout << "\n\nelapsed    : " << 1000 * elapsed << " msec" << endl;
      cout << "elapsed_raw   : " << 1000 * (t2 - t1) << "msec" << endl;
      cout << "elapsed_foc   : " << 1000 * (t3 - t2) << "msec" << endl;
      cout << "elapsed_wit   : " << 1000 * (t4 - t3) << "msec" << endl;
      cout << "avg elapsed: " << 1000 * tot_elapsed / elapsed_cnt << " msec" << endl;
      cout << "max elapsed: " << 1000 * max_elapsed << " msec" << endl;
      printf("\n");

      // Output the ft_response data
      
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

      disp_cnt = 0;
    }
  */
}

void Netboxrec::operate()
{
  // need a while loop here
  recvStream();
  writeFile();
}