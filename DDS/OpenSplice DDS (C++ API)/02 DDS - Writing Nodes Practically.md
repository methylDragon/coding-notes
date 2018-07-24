# OpenSplice DDS Crash Course

Author: methylDragon  
Let's get acquainted with DDS, fast!  
Compiling my notes from a whole variety of sources (mainly https://www.slideshare.net/Angelo.Corsaro/getting-started-with-dds-in-c-java-and-scala and http://download.prismtech.com/docs/Vortex/pdfs/OpenSplice_DDSTutorial.pdf)

------

## Pre-Requisites

### Assumed knowledge

- C++! We're using the C++ API for OpenSplice DDS. Read my C++ tutorial! It's only 5 parts long, and you really only need like the first three, well four parts...
- Install OpenSplice DDS: https://github.com/grassjelly/opensplice-install



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)  
2. [Writing OpenSplice DDS Nodes](#2)    
   2.1   [Minimal Version (Not Recommended)](#2.1)    
   2.2   [Abstracted Version (Recommended)](#2.2)    




## 1. Introduction <a name="1"></a>

So you roughly get what's going on with DDS now. Let's actually get to writing nodes effectively and efficiently!

We're going to be using custom message types as well!



## 2. Writing OpenSplice DDS Nodes <a name="2"></a>

### 2.1 Minimal Version (Not Recommended) <a name="2.1"></a>

[go to top](#top)

Let's base this off of a pre-existing codebase: https://github.com/grassjelly/opensplice_minimal

You'll find the simplest starter-code version of the project in the resources folder. I've edited out extrenuous information and added to the README.md. **You may use the README.md in the package for further guidance.**

It is important to note that just this minimal version already abstracts a bit of important DDS configurations and commands, exposing them as methods for CheckStatus and DDSEntityManager (in the src folder.) Things like memory freeing and a bit of optimisation.

However, I **strongly recommend** that you use the [abstracted version](#2.2) of this tutorial so you avoid most of the boilerplate code.

To use it, follow the following steps:

1. Edit the idl file in the idl folder. (You can call it anything you want.)

   - Define the message types you'd like to use in the form of structs and modules

   - You MAY define multiple message types in that single idl file!

   - Modules will define namespaces (optional)

   - Structs will define the message type name, and the contents will be the contents of the message

     ```idl
     module HelloWorld
     {
         struct Msg
         {
            /** User ID */
            long userID;
            /**  message */
            string message;
         };
         #pragma keylist Msg userID
     };
     
     struct standaloneMsg
     {
        /** User ID */
        long userID;
        /**  message */
        string message;
     };
     #pragma keylist standaloneMsg userID
     ```

2. Generate the message headers using `./gencode` (Remember to source your Opensplice DDS directory!!)

3. Edit your src code

   - Leave DDSEntityManager and CheckStatus **ALONE**
   - You will need to leave the boilerplate code in (like create participant, type creation, etc.)
   - Be sure to include your message headers that were generated! (In this case it was ccpp_HelloWorld.h), but it'll actually be ccpp_<message_type_name>.h)

   ```c++
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
   ```

4. Edit the CMakeLists.txt file

   - Specifically changing the name of the included .cpp files for the messages

     ```cmake
     ADD_LIBRARY(GEN_SRC
       gen/<NAME>.cpp
       gen/<NAME>Dcps.cpp
       gen/<NAME>Dcps_impl.cpp
       gen/<NAME>SplDcps.cpp
     )
     ```

   - Also remember to add in the executables to be compiled! So for example, if you had an executable called pub, then add in these lines.

     ```cmake
     ADD_EXECUTABLE (pub 
         src/pub.cpp
     )
     
     TARGET_LINK_LIBRARIES (pub
         GEN_SRC
         MGR_SRC
         ${OpenSplice_LIBRARIES}
      )
     ```

5. Run `./compile` to compile the code.

6. Then you can go try out the executables by executing them from the build directory

   - Again, remember to source your Opensplice DDS directory!
   - `./build/pub`



### 2.2 Abstracted Version (Recommended) <a name="2.2"></a>

[go to top](#top)

Let's base this off of a pre-existing codebase: https://github.com/grassjelly/opensplice_boilerplate

You'll find the simplest starter-code version of the project in the resources folder. I've edited out extrenuous information and added to the README.md. **You may use the README.md in the package for further guidance.**

The reason why I went through the minimal way of doing things first is to allow you to have a greater appreciation of why the abstracted way of doing things is better, and to also give you some background about what's actually going on.

This version uses a codebase that's designed to allow for multiple publishers and subscribers to be easily added to your source code with minimal extra writing, and exposes a ROS-like interface for the publishers and subscribers.

To use it, follow the following steps:

1. Edit the idl file in the idl folder. (You can call it anything you want.)

   - Define the message types you'd like to use in the form of structs and modules

   - You MAY define multiple message types in that single idl file!

   - Modules will define namespaces (optional)

   - Structs will define the message type name, and the contents will be the contents of the message

     ```idl
     struct Template
     {
         long userID;
         string message;
     };
     #pragma keylist Template userID
     
     struct Sensor
     {
         long userID;
         string message;
     };
     #pragma keylist Sensor userID
     ```

2. Generate the message headers using `./gencode` (Remember to source your Opensplice DDS directory!!)

3. Edit your src code

   - The boilerplate header files should have abstracted away most of the fluff!
   - Be sure to include your message boilerplate headers that were generated! (In this case it was Sensor.h, and Template.h) As well as the Participant header, which will greatly help with abstraction!
   - Notice that there isn't any more random hard to understand code, no more event handlers, topic creation, etc.! All of it is abstracted away, and you can just deal with the publisher and subscriber!

   ```c++
   #include "Participant.h"
   #include "Template.h"
   #include "Sensor.h"
   
   // This particular node publishes a Sensor msg, and reads a Template msg coming in from the other node!
   
   int main(int argc, char* argv[])
   {
       os_time delay_200ms = { 0, 200000000 };
       
       // Create participant
       // Note that the string argument is the Partition name, not the node name
       // It should be the same across all participants you intend to have communicate
       Participant par("HelloWorld example", 1);
   
       // Create pub-sub
       SensorMsg::Publisher pub(par, (char*)"HelloWorldData_Msg2");
       TemplateMsg::Subscriber sub(par, (char*)"HelloWorldData_Msg");
       
       for(;;){
           // Create msg
           Sensor testmsg;
           testmsg.userID = 1;
           testmsg.message = DDS::string_dup("Hello World");
           pub.publish(testmsg);
   
           // Read incoming msg
           sub.read();        
           for (DDS::ULong j = 0; j < sub.msg_list.length(); j++)
           {
               cout << "=== [Subscriber] message received :" << endl;
               cout << "    userID  : " << sub.msg_list[j].userID << endl;
               cout << "    Message : \"" << sub.msg_list[j].message << "\"" << endl;
           }    
       }
       
       // Delete publisher
       pub.kill();
       sub.kill();
       
       // Delete participant
       par.kill();
       
       return 0;
   }
   ```

4. Edit the CMakeLists.txt file

   - Add the extra lines to include the new message type definitions that were generated!

     ```cmake
     include(lib/boilerplate/types/<NEW_MSG_TYPE_NAME>/CMakeLists.txt) # Here!
     
     ADD_LIBRARY (MGR_SRC
       lib/boilerplate/template/CheckStatus.cpp
       lib/boilerplate/template/Participant.cpp
       ${<NEW_MSG_TYPE_NAME>} # Here!
     )
     ```

   - Also remember to add in the executables to be compiled! So for example, if you had an executable called pub, then add in these lines.

     **NOTE:** If you want to use these nodes with ROS in a catkin workspace, please prepend all instances of GEN_SRC and MGR_SRC with some unique identifier (like the package name), so as to avoid namespace clashes and allow everything to compile smoothly. Beyond that, it's just going to require strategic splicing of the DDS and ROS CMakeLists.txt files.

     ```cmake
     ADD_EXECUTABLE (pubsub_1 
         src/pubsub_1.cpp
     )
     
     TARGET_LINK_LIBRARIES (pubsub_1
         GEN_SRC
         MGR_SRC
         ${OpenSplice_LIBRARIES}
      )
      
      ADD_EXECUTABLE (pubsub_2 
         src/pubsub_1.cpp
     )
     
     TARGET_LINK_LIBRARIES (pubsub_2
         GEN_SRC
         MGR_SRC
         ${OpenSplice_LIBRARIES}
      )
     ```

5. Run `./compile` to compile the code.

6. Then you can go try out the executables by executing them from the build directory

   - Again, remember to source your Opensplice DDS directory!
   - `./build/pubsub_1` `./build/pubsub_2`



```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\     
```

​        

------

[![Yeah! Buy the DRAGON a COFFEE!](../../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)