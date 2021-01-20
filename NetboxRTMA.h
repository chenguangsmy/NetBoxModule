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

using std::cerr;
using std::endl;
using std::ofstream;

class NetftRTMA
{
    // variables;
public:
    string fname;
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

    char data_dir[MAX_DATA_DIR_LEN];
    char subject_name[TAG_LENGTH];
    int session_num;
    char file_name[20]; //seperation file name
    string file_dir;
    bool flag_sconfig;
    bool flag_xmconfig;
    bool flag_sent;
    bool got_msg;
    CMessage inMsg;

protected:
    int argc;
    char **argv;
    int i,j;
    // functions;
public:
    // connectTo();
    void receive();
    FLAGS respond(RESPONSE *, FLAGS);
    void updateMsg(RESPONSE *);
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
        flag_sent = true;
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

        //keep_going = true;
    }

    // deconstruction;
    ~NetftRTMA()
    {
        //clean the space malloced.
    }
};
void NetftRTMA::updateMsg(RESPONSE *forceData)
{
    i = 0;
    // only package the first message in a set of NUM_SAMPLES
    // ** dangerous here! alter to mutex_lock and unlock, besides, use a structure!
    raw_force_data.sample_header = sample_gen.sample_header;
    force_data.sample_header = sample_gen.sample_header;
    // set these variables private and visit them using function

    raw_force_data.rdt_sequence = forceData->rdt_sequence;
    raw_force_data.ft_sequence = forceData->ft_sequence;
    raw_force_data.status = forceData->status;
    for (j = 0; j < 6; j++)
    {
        force_data.data[j] = forceData->FTData[j];
        force_data.offset[j] = forceData->FTAvg[j];
    }

    force_data.rdt_sequence = raw_force_data.rdt_sequence;
    force_data.ft_sequence = raw_force_data.ft_sequence;
    force_data.status = raw_force_data.status;
    for (j = 0; j < 6; j++)
    {
        raw_force_data.data[j] = force_data.data[j] + force_data.offset[j];
    }
}
void NetftRTMA::receive()
{
    printf("netRTMA: enter receieve\n");
    //got_msg = mod.ReadMessage(&inMsg, 0.01);
    got_msg = mod.ReadMessage(&inMsg, 0.0);
    printf("netRTMA: enter receieve ln 2\n");
    if (got_msg == true)
        flag_sent = false;
}
FLAGS NetftRTMA::respond(RESPONSE *froceData, FLAGS flag)
{
    printf("netRTMA: enter respond function\n");
    if (inMsg.msg_type == MT_MOVE_HOME)
    {
        printf("MT_MOVE_HOME: update Avg. \n");
        flag.UpdateAvg = true;
        inMsg.GetData(&tsc);
    }
    else if (inMsg.msg_type == MT_SESSION_CONFIG)
    {
        printf("MT_SESSION_CONFIG. \n");
        inMsg.GetData(&ssconfig);
        strcpy(data_dir, ssconfig.data_dir);
        file_dir = data_dir;
        flag_sconfig = true;
    }
    else if (inMsg.msg_type == MT_XM_START_SESSION)
    {
        printf("MT_XM_START_SESSION receieved! \n");
        inMsg.GetData(&stsession);
        strcpy(subject_name, stsession.subject_name);
        session_num = stsession.calib_session_id;
        sprintf(file_name, "%s%d.csv", subject_name, session_num);
        cout << "filename: " << file_name << endl;
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
    else if (inMsg.msg_type == MT_SAMPLE_GENERATED & flag_sent == false)
    {
        inMsg.GetData(&sample_gen);
        // package message here!
        updateMsg((RESPONSE *)froceData);
        //printf("        send data! \n");
        mod.SendMessage(&RawForceSensorDataMsg);
        mod.SendMessage(&ForceSensorDataMsg);
        flag_sent == true;
    }
    else if (inMsg.msg_type == MT_EXIT)
    {
        if ((inMsg.dest_mod_id == 0) || (inMsg.dest_mod_id == mod.GetModuleID()))
        {
            printf("got exit!\n");
            mod.SendSignal(MT_EXIT_ACK);
            mod.DisconnectFromMMM();
            //keep_going = false;

            //netrec->closeFile();  ////.....
            flag.CloseFile = true;
            //fwriting = false;
            //netrec->stopStream();  //.....
            flag.stopStream = true;

            //break;
        }
    }

    // task conditions logic
    if (flag_sconfig & flag_xmconfig)
    { //not writing file, but receieved config, open file
        // concern: if a session has longer (more than a session), will this still work?
        printf("before enter fileInit()\n");
        cout << file_name << endl;
        fname = file_dir + '/' + file_name;
        cout << "fname should be" << fname << endl;
        //netrec->fileInit(fname.c_str());  //.....!!!!!!!!!!!!!!???????
        flag.FileInit = true;
        //netrec->sendrequest();            //.....
        flag.SendRequest = true;
        printf("file initialized\n");
        //reset flags
        flag_sconfig = false;
        flag_xmconfig = false;
        //fwriting = true;
    }
    //  cout<<"end of NetftRTMA::respond"<<endl;
    return flag;
}