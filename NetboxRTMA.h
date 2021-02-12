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

#include "Netboxrec.h"
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
    bool is_send;

    bool got_msg;
    CMessage inMsg;

protected:
    int argc;
    char **argv;
    int i,j;
public:
    bool flag_exit;
    void receive();
    FLAGS respond(RESPONSE *, FLAGS);
    void updateMsg(RESPONSE );
    void respondr(RESPONSE *, Netboxrec *);
    //private:
    //protected:
public:
    NetftRTMA(int argc, char **argv) : argc(argc), argv(argv),
        ForceSensorDataMsg(MT_FORCE_SENSOR_DATA), RawForceSensorDataMsg(MT_RAW_FORCE_SENSOR_DATA)
    {
        printf("NetftRTMA construction!\n");
        // variables
        got_msg = false;
        flag_sconfig = false;
        flag_xmconfig = false;
        is_send = false;
        flag_exit = false;
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
        mod.Subscribe(MT_TASK_STATE_CONFIG);
        mod.Subscribe(MT_XM_START_SESSION);
        mod.Subscribe(MT_DENSO_MOVE_COMPLETE);
    }

    // deconstruction;
    ~NetftRTMA()
    {
        //clean the space malloced.
    }
};
void NetftRTMA::updateMsg(RESPONSE forceData)
{
    // only package the first message in a set of NUM_SAMPLES
    raw_force_data.sample_header = sample_gen.sample_header;
    force_data.sample_header = sample_gen.sample_header;
    // set these variables private and visit them using function

    raw_force_data.rdt_sequence = forceData.rdt_sequence;
    raw_force_data.ft_sequence = forceData.ft_sequence;
    raw_force_data.status = forceData.status;
    forceData.time = forceData.time; 
    for (j = 0; j < 6; j++)
    {
        force_data.data[j] = forceData.FTData[j];
        force_data.offset[j] = forceData.FTAvg[j];
    }

    force_data.rdt_sequence = raw_force_data.rdt_sequence;
    force_data.ft_sequence = raw_force_data.ft_sequence;
    force_data.status = raw_force_data.status;
    force_data.time = raw_force_data.time;
    for (j = 0; j < 6; j++)
    {
        raw_force_data.data[j] = force_data.data[j] + force_data.offset[j];
    }
    // set data
    RawForceSensorDataMsg.SetData(&raw_force_data, sizeof(raw_force_data));
    ForceSensorDataMsg.SetData(&force_data, sizeof(force_data));
}
void NetftRTMA::receive()
{
    got_msg = mod.ReadMessage(&inMsg, 0.05);
}

void NetftRTMA::respondr(RESPONSE *froceData, Netboxrec *netrec)
{
    //if (inMsg.msg_type == MT_MOVE_HOME)
    if (inMsg.msg_type == MT_DENSO_MOVE_COMPLETE)
    {
        printf("MT_MOVE_HOME: update Avg. \n");
        netrec->updateAvg();
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
        sprintf(file_name, "%sFT%d.csv", subject_name, session_num);
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
    else if (inMsg.msg_type == MT_SAMPLE_GENERATED & is_send == true)
    {
        inMsg.GetData(&sample_gen);
        // package message, each struct element to message.
        updateMsg(*froceData); 
        printf("SAMPLE_GENERATED: Send Force Data %03d to %03d \n", froceData->rdt_sequence, force_data.rdt_sequence);
        mod.SendMessage(&RawForceSensorDataMsg);
        mod.SendMessage(&ForceSensorDataMsg);
    }
    else if (inMsg.msg_type == MT_EXIT)
    {
		flag_exit = true;
        if ((inMsg.dest_mod_id == 0) || (inMsg.dest_mod_id == mod.GetModuleID()))
        {
            printf("got exit!\n");
            mod.SendSignal(MT_EXIT_ACK);
            mod.DisconnectFromMMM();
            //keep_going = false;
			netrec->setStreamStop(); // no longer stream, influencde the next thread
            netrec->sendRequestStop();   // sending signal of stop stream
            
			netrec->closeFile();
        }
    }
    else if (inMsg.msg_type == MT_TASK_STATE_CONFIG)
    {
      //printf("M: MT_TASK_STATE_CONFIG \n");
      MDF_TASK_STATE_CONFIG task_state_data;
      inMsg.GetData( &task_state_data);
      switch(task_state_data.id)
      {
        case 1:   // set joint center and endpoint center
          cout << " ST 1, ";
          is_send = false;
          break;
        case 2: // Present
          cout << " ST 2, ";
          is_send = true;
          break;
        case 3: //ForceRamp
          cout << " ST 3, ";
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
    // task conditions logic
    if (flag_sconfig && flag_xmconfig)
    { //not writing file, but receieved config, open file
        // concern: if a session has longer (more than a session), will this still work?
        printf("before enter fileInit()\n");
        cout << file_name << endl;
        fname = file_dir + '/' + file_name;
        cout << "fname should be" << fname << endl;

        netrec->fileInit(fname);

        netrec->sendRequestStart();
        //reset flags
        flag_sconfig = false;
        flag_xmconfig = false;
        netrec->setStreamStart(); 
        //fwriting = true;
    }
    //  cout<<"end of NetftRTMA::respond"<<endl;
    //return flag;
}
