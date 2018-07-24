// -- Std C/C++ Include
#include <string>
#include <sstream>
#include <iostream>

#include "ccpp_HelloWorld.h"
#include <thread>         // std::thread, std::this_thread::sleep_for
#include <chrono> 
#include "DDSEntityManager.h"

using namespace DDS;
using namespace HelloWorld;


int main(int argc, char* argv[])
{
    os_time delay_2ms = { 0, 2000000 };
    os_time delay_200ms = { 0, 200000000 };
    MsgSeq msgList;
    SampleInfoSeq infoSeq;

    DDSEntityManager mgr;

    // create domain participant
    mgr.createParticipant("HelloWorld example");

    //create type
    MsgTypeSupport_var mt = new MsgTypeSupport();
    mgr.registerType(mt.in());

    //create Topic
    char topic_name[] = "HelloWorldData_Msg";
    mgr.createTopic(topic_name);

    //create Subscriber
    mgr.createSubscriber();

    // create DataReader
    mgr.createReader();

    DataReader_var dreader = mgr.getReader();
    MsgDataReader_var HelloWorldReader = MsgDataReader::_narrow(dreader.in());
    checkHandle(HelloWorldReader.in(), "MsgDataReader::_narrow");

    cout << "=== [Subscriber] Ready ..." << endl;

    bool closed = false;
    ReturnCode_t status =  - 1;
    int count = 0;
    while (!closed && count < 1500) // We dont want the example to run indefinitely
    {
    status = HelloWorldReader->take(msgList, infoSeq, LENGTH_UNLIMITED,
        ANY_SAMPLE_STATE, ANY_VIEW_STATE, ANY_INSTANCE_STATE);
    checkStatus(status, "msgDataReader::take");
    for (DDS::ULong j = 0; j < msgList.length(); j++)
    {
        cout << "=== [Subscriber] message received :" << endl;
        cout << "    userID  : " << msgList[j].userID << endl;
        cout << "    Message : \"" << msgList[j].message << "\"" << endl;
        closed = true;
    }
    status = HelloWorldReader->return_loan(msgList, infoSeq);
    checkStatus(status, "MsgDataReader::return_loan");
    os_nanoSleep(delay_200ms);
    ++count;
    }

    os_nanoSleep(delay_2ms);

    //cleanup
    mgr.deleteReader();
    mgr.deleteSubscriber();
    mgr.deleteTopic();
    mgr.deleteParticipant();

    return 0;
}