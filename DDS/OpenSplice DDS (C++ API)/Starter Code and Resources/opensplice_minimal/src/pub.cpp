
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
  os_time delay_1s = { 1, 0 };
  DDSEntityManager mgr;

  // create domain participant
  mgr.createParticipant("HelloWorld example");

  //create type
  MsgTypeSupport_var mt = new MsgTypeSupport();
  mgr.registerType(mt.in());

  //create Topic
  char topic_name[] = "HelloWorldData_Msg";
  mgr.createTopic(topic_name);

  //create Publisher
  mgr.createPublisher();

  // create DataWriter :
  // If autodispose_unregistered_instances is set to true (default value),
  // you will have to start the subscriber before the publisher
  bool autodispose_unregistered_instances = false;
  mgr.createWriter(autodispose_unregistered_instances);

  // Publish Events
  DataWriter_var dwriter = mgr.getWriter();
  MsgDataWriter_var HelloWorldWriter = MsgDataWriter::_narrow(dwriter.in());

  Msg msgInstance; /* Example on Stack */
  msgInstance.userID = 1;
  msgInstance.message = DDS::string_dup("Hello World");
  cout << "=== [Publisher] writing a message containing :" << endl;
  cout << "    userID  : " << msgInstance.userID << endl;
  cout << "    Message : \"" << msgInstance.message << "\"" << endl;

  ReturnCode_t status = HelloWorldWriter->write(msgInstance, DDS::HANDLE_NIL);
  checkStatus(status, "MsgDataWriter::write");
  os_nanoSleep(delay_1s);

  /* Remove the DataWriters */
  mgr.deleteWriter();

  /* Remove the Publisher. */
  mgr.deletePublisher();

  /* Remove the Topics. */
  mgr.deleteTopic();

  /* Remove Participant. */
  mgr.deleteParticipant();

  return 0;
}
