# opensplice_boilerplate

This package is a slightly edited version of: https://github.com/grassjelly/opensplice_boilerplate

This package allows a user to create multiple instances of publishers and subscribers within a single node. The API supports instances of different message types defined in an IDL file.

Use the src and idl files that currently exist as starter code for your projects!



## Adding new message types:

#### 1.  Use the .idl file in the idl folder as a reference to write new message types (as structs)

#### 2.  Generate the message headers:

They will go into /gen

```bash
$ ./gencode
```

- (No need to delete the gen folder, the script will remove/replace pre-existing files there)
- (Remember to source your Opensplice DDS directory!)

This script will generate:
- The message headers for the idl file you have created. 
- Boilerplate code specific to the message type so you can use the API.
- A CMakeLists.txt for each message type boilerplates. This must be included in the main CMakeLists.txt so you can use it in the API. See No.3 .

**Specifically, the generated code files are:**

- All code in the /gen folder
- All code in the /lib/boilerplate/types folder

#### 3.  Add the generated code and boilerplates in the main CMakeLists.txt:

    include(lib/boilerplate/types/<NEW_MSG_TYPE_NAME>/CMakeLists.txt)
    
    ADD_LIBRARY (MGR_SRC
      lib/boilerplate/template/CheckStatus.cpp
      lib/boilerplate/template/Participant.cpp
      ${<NEW_MSG_TYPE_NAME>}
    )

**NOTE:** If you intend to use this as a 'ROS package' (building the package with catkin_make), i.e. if you want rosrun to run it, ensure that you prepend all MGR_SRCs and GEN_SRCs with the NAME OF THE PACKAGE, or some unique identifier to prevent library namespace clashes.

Otherwise once you have more than one package, the multiple MGR_SRC and GEN_SRC mentions WILL break the compilation!



## Using the API:

#### 1. Include your new message type's header files:

    #include "<NEW_MSG_TYPE_NAME>.h"

#### 2. Create participant:

The partition name and domain ID **MUST be the same** across ALL NODES that are meant to communicate to each other. It is **not the node name.**

    Participant par("PARTITION_NAME", <DOMAIN_ID>);

#### 3. Creating a Publisher. Remember to add "Msg" when calling the namespace

Like so: `<NEW_MSG_TYPE_NAME>Msg:`

(Eg. SensorMsg, TemplateMsg, etc.)

    <NEW_MSG_TYPE_NAME>Msg::Publisher pub(par, (char*)"example_topic_name");

#### 4. Publish:

    //create message object
    <NEW_MSG_TYPENAME> testmsg;
    
    //populate the data
    testmsg.userID = 1;
    testmsg.message = DDS::string_dup("Hello World");
    
    //publish testmsg
    pub.publish(testmsg);

#### 5. Creating a Subscriber. Remember to add "Msg" when calling the namespace

Same deal here

    <NEW_MSG_TYPE_NAME>Msg::Subscriber sub(par, (char*)"example_topic_name");

#### 6. Subscribing and receiving data:

    sub.read();        
    for (DDS::ULong j = 0; j < sub.msg_list.length(); j++)
    {
      cout << "=== [Subscriber] message received :" << endl;
      cout << "    userID  : " << sub.msg_list[j].userID << endl;
      cout << "    Message : \"" << sub.msg_list[j].message << "\"" << endl;
    }    


## Notes:

- You can have MULTIPLE publishers and subscribers for **DIFFERENT** topics/message types associated with one domain participant!
- You can have multiple domain participants as long as they're spread across different domain IDs. This will allow you to have multiple 'subnets' since only participants belonging to the same domain can communicate!