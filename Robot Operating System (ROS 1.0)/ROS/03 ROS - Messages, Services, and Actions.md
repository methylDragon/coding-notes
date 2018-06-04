# ROS Messages, Services, and Actions

Author: methylDragon  
Fairly comprehensive ROS crash course!  
I'll be adapting it from the ROS Tutorials:http://wiki.ros.org/ROS/Tutorials    
and ETHz http://www.rsl.ethz.ch/education-students/lectures/ros.html    

------

## Pre-Requisites

- A system with Ubuntu 16.04 installed (no other versions!)
- Linux
- Python 3 and/or C++



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)    
   1.1   [Introduction](#1.1)    
   1.2   [ROS Packages](#1.2)    
   1.3   [Nodes](#1.3)    
   1.4   [Parameters](#1.4)    
2. [Basics of Writing Nodes](#2)    
   2.1   [Introduction](#2.1)    
   2.2   [rospy](#2.2)    
   2.3   [rospy: hello_world](#2.3)    
   2.4   [rospy: Basic Publisher](#2.4)    
   2.5   [rospy: Basic Subscriber](#2.5)    
   2.6   [rospy: Parameters](#2.6)    
   2.7   [Making and building the rospy package](#2.7)    
   2.8   [Extra rospy tips](#2.8)    
   2.9   [roscpp](#2.9)    
   2.10 [roscpp: Concepts](#2.10)    
   2.11 [roscpp: hello_world](#2.11)    
   2.12 [roscpp: Basic Publisher](#2.12)    
   2.13 [roscpp: Basic Subscriber](#2.13)    
   2.14 [roscpp: Parameters](#2.14)    
   2.15 [Making and building the roscpp package](#2.15)    
3. [STILL WORKING ON THIS](#3)    
   3.1   [ROS Workspaces](#3.1)    
   3.2   [rosbash](#3.2)    
   3.3   [roscore (ROS Master)](#3.3)    
   3.4   [Using ROS](#3.4)    
   3.5   [A Simple ROS Example](#3.5)    
   3.6   [Visualising the ROS Graph](#3.6)    
   3.7   [roslaunch](#3.7)    
   3.8   [Launch Files](#3.8)    
   3.9   [Gazebo (Simulation)](#3.9)     



## 1. Introduction <a name="1"></a>

So now you know how to write nodes. Ever wondered if you could do more? Of course you can!

You can create **custom messages**, create **nodes just for serving requests**, or nodes that **carry out actions, and provide feedback.**

We're going to talk about **messages, services, and actions!**



## 2. Messages <a name="2"></a>

### 2.1. Introduction <a name="2.1"></a>

[go to top](#top)

Messages are what are sent to ROS nodes via topics! They are the subject that is published and sent to subscribers!

You can create messages very easily, because ROS has a couple of macros that dynamically generate the language-specific message related code for you! All you need to do, is create the **.msg file**.

> [msg](http://wiki.ros.org/msg): msg files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.
>
> (http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)



### 2.2. Messages <a name="2.2"></a>

[go to top](#top)

#### **Eligible Message Field Types**

Msg files are text files that consist of a single field per line. These are the eligible types:

- int8, int16, int32, int64 (plus uint*)
- float32, float64
- string
- time, duration
- other msg files
- variable-length array[] and fixed-length array[C]
- the special Header type

> The **Header** type consists of a **timestamp** and **coordinate frame.** A lot of packages use them, and it normally appears as the first line of a .msg file.



#### **Example .msg file**

```
  Header header
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose
  geometry_msgs/TwistWithCovariance twist
```



### 2.3 rosmsg <a name="2.3"></a>

[go to top](#top)

ROS has a bunch of command line tools to help with messages as well!

```shell
# Show displays the contents of the corresponding message file
$ rosmsg show package_name message_name

# If you don't know the package, rosmsg will search for a matching message file!
$ rosmsg show message_name

# You can even use it in combination with rostopic to display type info about the topic!
$ rostopic type /topic_name | rosmsg show


# List displays a list of all messages
$ rosmsg list

# Package displays a list of all messages in a package
$ rosmsg package package_name

# Packages displays a list of all packages with messages
$ rosmsg packages

```



### 2.4 Creating Custom Messages  <a name="2.4"></a>

[go to top](#top)

Let's try making some custom messages!

> Reminder: To create a new package,
>
> ```shell
> $ cd <your_workspace_directory>/src
> $ catkin_create_pkg package_name rospy <any other dependencies, including standard ones!>
> 
> # Eg. catkin_create_pkg basic_pub_sub rospy std_msgs
> ```

**1. Go to your package directory**

```shell
$ roscd msg_example_msgs # Using a message-only package is good practice!
$ mkdir msg # Make a msg folder to keep your message files

$ cd msg
$ touch My_msg.msg # Capitalising the first letter is the convention!
```

**2. Then open that .msg file in your favourite text editor** and HAVE AT IT! (Write your data types and names)

```
Header header
string name
uint8 dragon_rating
```

**3. Then go to package.xml** and **append these two lines**

```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

**4. Then say hello to our ~~nemesis~~ good old friend CMakeLists.txt**

Ensure these lines are present

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(msg_example_msgs)

# Standard dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# Your message files!
add_message_files(
  FILES
  My_msg.msg # <-- Like this one!
)

# You need to include this in as well
# Call it BEFORE catkin_package
generate_messages(
  DEPENDENCIES
  std_msgs # You might have additional message files from other packages
)

# Additional catkin dependencies
catkin_package(CATKIN_DEPENDS
  message_runtime
)
```

> NOTE:
>
> When you are compiling a package that requires message files from a separate message package, be sure to add
>
> ```cmake
> add_dependencies(CURRENT_PACKAGE MESSAGE_PACKAGE_generate_messages_cpp)
> ```
>
> Otherwise it **will** fail!

**5. Then go back to your workspace root, and rebuild!**

```shell
$ roscd msg_example_msgs
$ cd ../..
$ catkin_make # or catkin_make install, see what works
```



### 2.5 Using Custom Messages <a name="2.5"></a>

[go to top](#top)

In the last tutorial part, we learnt how to use the **rospy** and **roscpp** client libraries to write nodes that use in-built messages!

Now I'll show you how to use them to include/import custom messages!

> Our message file is structured as such
>
> ```
> Header header
> string name
> uint8 dragon_rating
> ```

#### **rospy**

Including the message

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ^^^ The encoding line is just because I want to have a cute emoticon ヾ(°∇°*)

import rospy
from msg_example_msgs.msg import My_msg

# Create the message object to publish
msg = My_msg()
```

Testing it with a publisher

```python
# Then to use it~
def talker():
    # My_msg is the topic type (aka the message file the topic takes)
    pub = rospy.Publisher("msg_example", My_msg, queue_size = 10)
    
    rospy.init_node("msg_example_node", anonymous = True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now() # Since our message has a header
        # The header sequence value will increase once a subscriber is connected
        
        msg.name = "methylDragon"
        msg.dragon_rating = 10
        
        rospy.loginfo("FIND MY OUTPUT ON /msg_example ! ヾ(°∇°*)")
        
        pub.publish(msg)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

#### **roscpp**

Including the message

```c++
#include <ros/ros.h>
#include <msg_example_msgs/My_msg.h>

msg_example_msgs::My_msg msg;
```

Testing it with a publisher

```c++
int main(int argc, char **argv)
{
    ros::init(argc, argv, "msg_example_node");
    ros::NodeHandle nh;

    ros::Publisher chatter_publisher = nh.advertise<msg_example_msgs::My_msg>("msg_example", 1);

    ros::Rate loopRate(10);

    unsigned int count = 0;

    while (ros::ok())
    {
        msg_example_msgs::My_msg msg; // Make a new message object
        msg.header.stamp = ros::Time::now();
        
        msg.name = "methylDragon";
        msg.dragon_rating = 10;
        
        ROS_INFO_STREAM("       .     .\n"
                        "                                    .  |\\-^-/|  .\n"
                        "FIND MY OUTPUT ON /msg_example !   /| } O.=.O { |\\ \n");

        chatter_publisher.publish(msg);

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
```



Oh my goodness that was **exceedingly hard!**

> Note: If you're importing messages from other packages, append these lines to **package.xml**
>
> ```xml
> <build_depend>name_of_package_containing_custom_msg</build_depend>
> <exec_depend>name_of_package_containing_custom_msg</exec_depend>
> ```
>
> And this line to **CMakeLists.txt**
>
> ```cmake
> add_dependencies(your_program ${catkin_EXPORTED_TARGETS})
> 
> # And then add these lines if you're using roscpp
> find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs)
> include_directories(include ${catkin_INCLUDE_DIRS})
> 
> add_dependencies(your_program ${catkin_EXPORTED_TARGETS})
> add_dependencies(your_library ${catkin_EXPORTED_TARGETS})
> ```



## 3. Services <a name="3"></a>

### 3.1 Introduction <a name="3.1"></a>

[go to top](#top)

A ROS service is a special kind of topic that allows for two-way communication between nodes. (Specifically, request-response communication.)

A service is used by two types of nodes:

- The service server advertises the service, and listens for messages sent to the service
  - It then computes the response, and publishes its response to the service!
- The service client publishes to the service, calling it
  - Then, it subscribes to the service and waits for the response

The message types available for use with services are based off of the eligible .msg types, as shown in the previous section of this tutorial.

But the service messages are defined in a special type of file called a .srv file.

![3.1](./Images/3.1.png)

(Image source: ETHz)



### 3.2 Services  <a name="3.2"></a>

[go to top](#top)

#### **Eligible Service Field Types**

Exactly the same as messages!

Service files are just two message files 'smushed' into one!



#### **Example .srv file**

Recall the example .msg file:

```
  Header header
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose
  geometry_msgs/TwistWithCovariance twist
```

A .srv file is **similar** (it's just two message files smushed into one, one request, one response)

```
request
---
response
```

Eg:

```
int64 A
int64 B
---
int64 Sum
```



### 3.3 rosservice  <a name="3.3"></a>

[go to top](#top)

> NOTE: `rosservice` is **different** from `rossrv`, which does the same thing that `rosmsg` does for .srv files.

You can use `rosservice` in the same way `rostopic` is used to publish and subscribe to topics, except now you're **calling the service.**

```shell
# List available services
$ rosservice list

# Show the type of the service (what messages it takes)
$ rosservice type /service_name

# Call a service
$ rosservice call /service_name <argument_1> <argument_2> ...
```



### 3.4 Creating Services <a name="3.4"></a>

[go to top](#top)

Okay. Let's make a service file!

> Reminder: To create a new package,
>
> ```shell
> $ cd <your_workspace_directory>/src
> $ catkin_create_pkg package_name rospy <any other dependencies, including standard ones!>
> 
> # Eg. catkin_create_pkg basic_pub_sub rospy std_msgs
> ```

**1. Go to your package directory**

```shell
$ roscd srv_example
$ mkdir srv # Make a srv folder to keep your service files

$ cd srv
$ touch AddTwoInts.srv # Capitalising the first letter is the convention!
```

**2. Then open that .msg file in your favourite text editor** and HAVE AT IT! (Write your data types and names)

```
int64 A
int64 B
---
int64 Sum
```

**3. Then go to package.xml** and **append these two lines**

```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

**4. Then say hello to our ~~nemesis~~ good old friend CMakeLists.txt**

Ensure these lines are present (**IN ORDER!**)

```cmake
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs # <-- Add these
   message_generation # <-- Add these (even though you're generating services)
)

# Uncomment this block
add_service_files(
  FILES
  AddTwoInts.srv # <-- Add your srv files
)

# Uncomment this block (even though you're generating services)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime <and your other catkin dependencies> # <-- Add this 
)
```

**5. Then go back to your workspace root, and rebuild!**

```shell
$ roscd example_service_package
$ cd ../..
$ catkin_make # or catkin_make install, see what works
```



### 3.5 Using Custom Services <a name="3.5"></a>

[go to top](#top)

This one is a little more involved than just including the messages like we did in the previous part.

Let's use this service

AddTwoInts.srv

```
int64 A
int64 B
---
int64 Sum
```

Which should have generated a header file called AddTwoInts.h on build.



#### **rospy server**

>  Note: You import from your PACKAGE_NAME.srv !

```python
#!/usr/bin/env python

from srv_example.srv import *
import rospy

# Here's your service callback function
def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]" % (req.A, req.B, (req.A + req.B)))
    return AddTwoIntsResponse(req.A + req.B)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    
    # Here's your service hook (AddTwoInts is the service type/file)
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
```

#### **rospy client**

```python
#!/usr/bin/env python

import sys
import rospy
from srv_example.srv import *

# Service call handler (arguments are request.A, request.B, as x and y)
def add_two_ints_client(x, y):
    # Wait for service to become available
    rospy.wait_for_service('add_two_ints')
    
    try:
        # The service proxy handles the service call, like a temporary node!
        # add_two_ints is the service name
        # AddTwoInts is the service type/file
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum

    except rospy.ServiceException, e:
        print("Service call failed: %s" %  e)

# Invalid call reminder
def usage():
    return "Usage: %s [x y]" % sys.argv[0]

if __name__ == "__main__":
    
    # If the service call was properly formatted, proceed
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
        
    # Else, remind the user of the proper usage
    else:
        print(usage())
        sys.exit(1) # Terminate the script	

    print("Requesting %s+%s" % (x, y))
    print("%s + %s = %s" % (x, y, add_two_ints_client(x, y)))
```

> Remember to `chmod +x scripts/service_node_file.py` !



#### **roscpp server**

```c++
#include <ros/ros.h>
#include <srv_example/AddTwoInts.h>

// Callback function
bool add(srv_example::AddTwoInts::Request &request,
         srv_example::AddTwoInts::Response &response)
{
  response.Sum = request.A + request.B;
  
  ROS_INFO("request: x=%ld, y=%ld", (long int)request.B, (long int)request.B);
  
  ROS_INFO(" sending back response: [%ld]", (long int)response.Sum);
  
  return true;
}

int main(int argc, char **argv)
{
  // Start the node
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle nh;
  
  // Advertise the service
  ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
    
  // Spin up
  ros::spin();

  return 0;
}
```

#### **roscpp client**

```c++
#include <ros/ros.h>
#include <srv_example/AddTwoInts.h>
#include <cstdlib>

int main(int argc, char **argv) {
  ros::init(argc, argv, "add_two_ints_client");

  if (argc != 3) {
    ROS_INFO("Usage: %s [x y]", argv[0]);
    return 1;
}

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<srv_example::AddTwoInts>("add_two_ints");

srv_example::AddTwoInts service;
service.request.A = atoi(argv[1]);
service.request.B = atoi(argv[2]);

// Service callback function
// client.call(service) calls the client!
if (client.call(service)) {
  ROS_INFO("Sum: %ld", (long int)service.response.Sum);
}
else {
  ROS_ERROR("Failed to call service add_two_ints");
  return 1;
}

  return 0;
} 
```



#### **Testing your service**

```shell
# Terminal 1
$ roscore

# Terminal 2
$ rosrun srv_example srv_example_server

# Terminal 3
$ rosservice call /add_two_ints 1 3

# Expected outputs
# Terminal 2: Returning [1 + 3 = 4] (Or something along those lines)
# Terminal 3: Sum: 4

# NOTE: Remember to catkin_make and source your devel/setup.bash !
```



#### **argv refresher**

`sys.argv` is a list that contains the arguments passed to the script using the command line. It comes from the argc argv  (argument count, argument vector) concept in C++.

Basically, the elements of sys.argv are as such.

The **first** element is the name of the script being called

The elements **beyond** the first are the arguments passed to the script in order.

SO. This is why you see calls to sys.argv[1], sys.argv[2] for the 2 argument service call in the rospy service example.



### 1.5 Services <a name="1.5"></a>

[go to top](#top)



### 1.5 Services <a name="1.5"></a>

[go to top](#top)



### 1.5 Services <a name="1.5"></a>

[go to top](#top)



### 1.5 Services <a name="1.5"></a>

[go to top](#top)













## 2. Basics of Writing Nodes <a name="2"></a>

### 2.1 Introduction <a name="2.1"></a>

[go to top](#top)

It's time to begin!

There are two commonly used client libraries/APIs for ROS.

- **rospy** for Python
- **roscpp** for C++

Python is generally slower, but easier to write for than C++. You can mix nodes of the two different types, but mixing languages within a node is a little harder.



### 2.2 rospy <a name="2.2"></a>

[go to top](#top)

(ros - pee, ros_py!!)

Adapted from: http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber

>  Minimal projects can be found in the tutorial folder.

Remember that we're writing nodes here! So the hello world for this will be a 'hello world node', so to speak.

Ensure the following:

- You've created the workspace and sourced it
- These scripts are in your package's **src** directory
- You've built the scripts (read down for the tutorial for that)



### 2.3 rospy: hello_world <a name="2.3"></a>

[go to top](#top)

(You don't need a .py, the first line solves that issue)

```python
#!/usr/bin/env python
# The above line is MANDATORY

import rospy # Here's rospy!
from std_msgs.msg import String # This is how we import message types

def logger():
	
    # Initialise the node, call it logger
    # Anonymous means the node gets created as talker_serialnumber
    # (That prevents namespace clashes)
    # If there's a clash, the existing node will be booted off and forced to shutdown
    rospy.init_node('logger', anonymous = True)
    
    rate = rospy.Rate(10) # 10hz
    
    # As long as the node is not shutdown, keep running this
    while not rospy.is_shutdown():
        log_str = "HELLO WORLD"
        
        # ROS log is usually shown on the Terminal screen
        rospy.loginfo(log_str)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        logger()
    except rospy.ROSInterruptException:
        pass    
```



### 2.4 rospy: Basic Publisher <a name="2.4"></a>

[go to top](#top)

```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    # Create a publisher object
    # It publishes String messages to the topic 'chatter'
    pub = rospy.Publisher('chatter', String, queue_size = 10)
    
    rospy.init_node('talker', anonymous = True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass    
```



### 2.5 rospy: Basic Subscriber <a name="2.5"></a>

[go to top](#top)

```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():
    rospy.init_node('listener', anonymous = True)
    
    # Callback function gets run each time a message is received
    rospy.Subscriber("chatter", String, callback)

    # spin() keeps Python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```



### 2.6 rospy: Parameters <a name="2.6"></a>

[go to top](#top)

Source: http://wiki.ros.org/rospy/Overview/Parameter%20Server

**Get parameters** (`get_param`)

Dictionaries are returned

```python
global_name = rospy.get_param("/global_name/etc")
relative_name = rospy.get_param("relative_name/etc")
private_param = rospy.get_param('~private_name/etc')    
default_param = rospy.get_param('default_param/etc', 'default_value')

# Fetch a group (dictionary) of parameters
gains = rospy.get_param('gains')
p, i, d = gains['P'], gains['I'], gains['D']
```

**Set parameters** (`set_param`, `set_param_raw`)

```python
# Using yaml strings
rospy.set_param('a_string', 'baz')
rospy.set_param('~private_int', '2')
rospy.set_param('list_of_floats', "[1., 2., 3., 4.]")
rospy.set_param('bool_True', "true")
rospy.set_param('gains', "{'p': 1, 'i': 2, 'd': 3}")

# Using raw python objects
rospy.set_param_raw('a_string', 'baz')
rospy.set_param_raw('~private_int', 2)
rospy.set_param_raw('list_of_floats', [1., 2., 3., 4.])
rospy.set_param_raw('bool_True', True)
rospy.set_param_raw('gains', {'p': 1, 'i': 2, 'd': 3})

rospy.get_param('gains/P') #should return 1
```

**Check parameter existance** (`has_param`)

```python
if rospy.has_param('to_delete'):
    rospy.delete_param('to_delete')
```

**Delete parameter** (`delete_param`)

```python
try:
    rospy.delete_param('to_delete')
except KeyError:
    print "value not set"
```

**Search parameters** (`search_param`)

```python
# This gets us the closest matching parameter in terms of the namespace
# But the name has to match, of course
param_name = rospy.search_param('global_example')

v = rospy.get_param(param_name)
```



### 2.7 Making and building the rospy package <a name="2.7"></a>

[go to top](#top)

Pre-requisites:

- You have to have **created** and **sourced** your catkin workspace!

  - Create some workspace (example_ws)

  - Create a src directory inside that workspace

  - Go to the root of the workspace, and use `$ catkin_make`

  - Then `$ source devel/setup.bash`

**1. Initialise your package**

```shell
$ cd <your_workspace_directory>/src
$ catkin_create_pkg package_name rospy <any other dependencies, including standard ones!>

# Eg. catkin_create_pkg basic_pub_sub rospy std_msgs
```

**2. Populate src with your rospy scripts**

Or, if you want to go modular, put them in sub-directories!

> **Example folder structure:**
>
> my_catkin_ws
>
> - src
>   - CMakeLists.txt
>   - **YOUR ROSPY PACKAGE**
>     - CMakeLists.txt (This is the one you edit)
>     - package.xml (This too!)
>     - **setup.py**
>     - src
>       - **PUT YOUR PYTHON SCRIPTS HERE**

**3. Create setup.py**

#### **Setup.py**

^ Super important! Put it in the **root** of your **package**!

Example setup.py:

```python
#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    
    # State your package directories within /src here
    packages = ['package_1', 'package_2'],
    
    # Script locations
    scripts = ['scripts/script_name'],
    
    # root/src, basically
    package_dir = {'': 'src'},
    
    # Your Python dependencies (eg. 'serial')
    install_requires = ['python_module_1', 'python_module_2']
)

setup(**setup_args)
```

**4. Write your package description (package.xml)**

For more info, read the **catkin tutorial**

Example package.xml:

>  **NOTE:** Python dependencies are defined using the `<exec_depend>` tags. But using the name from the rosdistro_list!
>
> In so doing, it's slightly different from declaring Python dependencies in setup.py.
>
> Writing it this way allows catkin to install it for other people when they install via catkin

```python
<?xml version="1.0"?>
<package format="2">
  <name>basic_pub_sub</name>
  <version>0.0.0</version>
  <description>A minimal rospy basic pub-sub package!</description>

  <author email="methyldragon@gmail.com">methylDragon</author>
  <maintainer email="methyldragon@gmail.com">methylDragon</maintainer>
  <url type="website">http://github.com/methylDragon</url>

  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>

  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>

  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  
</package>
```

**5. Configure the build (CMakeLists.txt)**

For more info, read the **catkin tutorial**

Example CMakeLists.txt:

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(<package_name>)

find_package(catkin REQUIRED COMPONENTS
  rospy
  <other_dependencies>
)

# Enable python building
catkin_python_setup()

# Initialise the export variables
# Giving no arguments still initialises the variables (eg. CATKIN_PACKAGE_BIN_DESTINATION)
catkin_package()

# This is for installing SCRIPTS into the Install space
# Note: ONLY INSTALL THE EXECUTABLES YOU WANT TO BE ABLE TO ROSRUN!!
install(PROGRAMS
  <YOUR SOURCE_CODE DIRS HERE>
  <folders/SOURCE_CODE>
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

**6. Build the package**

- Go to the root of your workspace
- Run `catkin_make` and ensure no errors occured
- Source your workspace again `source devel/setup.bash`

**7. Verify the package**

Start a ROS master

```shell
$ roscore
```

Run your nodes

```shell
$ rosrun your_package_name node_name
```



#### **False Failures**

> You might find that the first time you run your package, the command will not autocomplete, because ROS takes awhile to find the package.
>
> To speed it along, either manually type out the rosrun command, or use `rospack profile` to rebuild the package tree.



### 2.8 Extra rospy tips <a name="2.8"></a>

[go to top](#top)

#### **Modularisation**

Read more: http://www.artificialhumancompanions.com/structure-python-based-ros-package/

> Minimal projects can be found in the tutorial folder.

It's always a good thing to **modularise** your code.

> Actually you can just name the folders whatever you want, but nodes and scripts is clear enough
>
> (Sometimes /nodes is called /bin)
>
> Nodes: Something you expect ROS to rosrun with
>
> Scripts: Any other kinds of scripts

---

> **Example folder structure:**
>
> - my_catkin_ws
>   - CMakeLists.txt
>   - src
>     - **YOUR_ROSPY_PACKAGE**
>       - CMakeLists.txt
>       - package.xml
>       - setup.py
>       - nodes
>         - node_1
>       - scripts
>         - script_1
>       - src
>         - python_package_1
>           - \_\_init\_\_.py
>           - node_source.py
>           - python_sub_package_1
>             - \_\_init\_\_.py
>             - node_sub_source_1.py
>             - node_sub_source_2.py

NOTE: Conventionally, the python package will be called the same name as the ROS package
> You must also use (and configure!) setup.py, which is the macro that will help Catkin locate the relevant Python files you'll want to use, as well as add your package to PYTHONPATH.

Where:

**python_package_1/\_\_init\_\_.py**

```python
#!/usr/bin/env python

from node_source import main
```

**node_1** (the executable node)

```python
#!/usr/bin/env python

# You can do this because of the __init__.py in my_pkg
from python_package_1 import main

if __name__== '__main__':
     main()
```

**node_source** (the code implementing the node)

```python
#!/usr/bin/env python

from python_sub_package_1 import node_sub_source_1

# Your implementing source code
# etc. etc.
```

(or, if you added an import statement in the sub_package's \_\_init\_\_.py,)

```python
#!/usr/bin/env python

from python_sub_package_1 import some_function

# + other source code
```

**setup.py**

```python
#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# No Python dependencies in this example

setup_args = generate_distutils_setup(
    packages=['my_pkg'],
    package_dir={'': 'src'},
)


setup(**setup_args)
```

**CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(my_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()
catkin_python_setup()

install(PROGRAMS
  nodes/node_1
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```



#### **More info about \_\_init\_\_.py**

Read here: https://timothybramlett.com/How_to_create_a_Python_Package_with___init__py.html

Or read the advanced section of the Python tutorial!



#### **A better way to publish**

Source: http://wiki.ros.org/rospy_tutorials/Tutorials/AdvancedPublishing

As we know, when you're trying to publish something

```python
pub.publish(some_string) # rospy knows to create a std_msg.msg.String
```

But what if the message takes in multiple arguments?

```shell
$ rosmsg show std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
```

Then you have to write it like this

```python
pub.publish(0.1, 0.2, 0.3, 0.4)

# But this is (in the words of the tutorial) kind of brittle!
# A better way is to actually use kwargs

pub.publish(a = 1.0)

# The rest will default to 0
```



### 2.9 roscpp  <a name="2.9"></a>

[go to top](#top)

Time to get started with C++!

>  Minimal projects can be found in the tutorial folder.

Remember that we're writing nodes here! So the hello world for this will be a 'hello world node', so to speak.

Ensure the following:

- You've created the workspace and sourced it
- These scripts are in your package's **src** directory
- You've built the scripts (read down for the tutorial for that)



### 2.10 roscpp: Concepts  <a name="2.10"></a>

[go to top](#top)

We'll be dealing with a bit more this time around than what we did in the rospy tutorial.

roscpp goes a tiny bit deeper!



#### **Node Handles**

Read more: http://wiki.ros.org/roscpp/Overview/NodeHandles

Node handles do several things:

- Handle **initialisation** and **shutdown**
- Handle communication with ROS (topics, services, parameters, etc.)



These node handles can exist in several types of namespaces:

- Public (default)
  - Eg: `nh_ = ros::NodeHandle();` (or `ros::NodeHandle nh_;`)
  - When looking for topics, resolves to: /namespace/topic
- Private
  - Eg: `nh_private_ = ros::NodeHandle("~");`
  - When looking for topics, resolves to: /namespace/node/topic
- Namespaced
  - Eg: `nh_rawr_ = ros::NodeHandle("rawr");`
  - When looking for topics, resolves to: /namespace/specific_name/topic
- Global
  - Eg: `nh_global_ = ros::NodeHandle("/");`
  - When looking for topics, resolves to: /topic



### 2.11 roscpp: hello_world  <a name="2.11"></a>

[go to top](#top)

```C++
#include <ros/ros.h> // Include the ROS main header file

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rawrer", ros::init_options::AnonymousName);
    // Initialise a node named rawrer, make it anonymous
    // Check the rospy hello_world for what anonymous means

    ros::NodeHandle node_handle; // This HANDLES all communications with ROS
    // Topics, services, parameters, etc

    ros::Rate loopRate(10); // Hz

    // Let's just have a nice incrementer here
    unsigned int count = 0;

    // As long as ROS is running, keep running
    while (ros::ok())
    { // ros::ok() checks for the status of ROS
        ROS_INFO_STREAM("Rawr " << count); // Log it!
        // Notice how we aren't using cout here! Use ROS_INFO and ROS_INFO_STREAM!

        ros::spinOnce(); // Calls all callbacks waiting to be called-back
        loopRate.sleep(); // Sleep according to the loopRate
        count++; // Aaaand increment our counter
    }

    return 0;
}
```



### 2.12 roscpp: Basic Publisher  <a name="2.12"></a>

[go to top](#top)

```c++
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char **argv)
{
    // New node called talker
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    // Publish onto the "chatter" topic with a queue size of 1
    ros::Publisher chatter_publisher = nh.advertise<std_msgs::String>("chatter", 1);

    ros::Rate loopRate(10);

    unsigned int count = 0;

    while (ros::ok())
    {
        std_msgs::String message; // Make a new message object
        std::ostringstream string_count;
        string_count << count;

        message.data = "Rawr " + string_count.str(); // Write to it
        ROS_INFO_STREAM(message.data);

        chatter_publisher.publish(message);

        ros::spinOnce();
        loopRate.sleep();
        count++;
    }

    return 0;
}
```





### 2.13 roscpp: Basic Subscriber  <a name="2.13"></a>

[go to top](#top)

```c++
#include <ros/ros.h> // Include the ROS main header file

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rawrer", ros::init_options::AnonymousName);
    // Initialise a node named rawrer, make it anonymous
    // Check the rospy hello_world for what anonymous means

    ros::NodeHandle node_handle; // This HANDLES all communications with ROS
    // Topics, services, parameters, etc

    ros::Rate loopRate(10); // Hz

    // Let's just have a nice incrementer here
    unsigned int count = 0;

    // As long as ROS is running, keep running
    while (ros::ok())
    { // ros::ok() checks for the status of ROS
        ROS_INFO_STREAM("Rawr " << count); // Log it!
        // Notice how we aren't using cout here! Use ROS_INFO and ROS_INFO_STREAM!

        ros::spinOnce(); // Calls all callbacks waiting to be called-back
        loopRate.sleep(); // Sleep according to the loopRate
        count++; // Aaaand increment our counter
    }

    return 0;
}
```



### 2.14 roscpp: Parameters  <a name="2.14"></a>

[go to top](#top)

First make a nodehandle: 

`ros::NodeHandle nh("~");` (For parameters, the nodehandle to use is normally **private**)



**Get parameters** (`getParam()`, `param()`)

`getParam()` returns a Boolean, but stores in the variable in the second argument

Returns **True** if a value was fetched

Returns **False** otherwise

`param()` works the same way, but if the parameter can't be retrieved, you can set a default value for it in that case.

```c++
std::string s;
int i;

// getParam()
// The param_name's fetched parameter value is stored in s!
nh.getParam("param_name", s);

// param()
nh.param("param_name_2", i, 42); // Default value is now 42

nh.param<std::string>("param_name_2", s, "default_val")
// Sometimes you need to include a hint
```

**Set parameters** (`setParam()`)

```c++
nh.setParam("param_name", "rawr");
```

**Check parameter existance** (`hasParam()`)

```c++
if (!nh.hasParam("my_param"))
{
	ROS_INFO("No param named 'my_param'");
}
```

**Delete parameter** (`deleteParam()`)

```python
nh.deleteParam("param_name");
```

**Search parameters** (`searchParam()`)

```c++
std::string param_name;
if (nh.searchParam("b", param_name))
{
	// If we found it, we can now query it using param_name
	int i = 0;

    nh.getParam(param_name, i);
}

else
{
	ROS_INFO("No param 'b' found in an upward search");
}
```



### 2.15 Making and building the roscpp package  <a name="2.15"></a>

[go to top](#top)

Pre-requisites:

- You have to have **created** and **sourced** your catkin workspace!
  - Create some workspace (example_ws)
  - Create a src directory inside that workspace
  - Go to the root of the workspace, and use `$ catkin_make`
  - Then `$ source devel/setup.bash`

**1. Initialise your package**

```shell
$ cd <your_workspace_directory>/src
$ catkin_create_pkg package_name roscpp <any other dependencies, including standard ones!>

# Eg. catkin_create_pkg basic_pub_sub roscpp std_msgs
```

**2. Populate src with your roscpp scripts**

Or, if you want to go modular, put them in sub-directories!

> **Example folder structure:**
>
> - my_catkin_ws
>   - src
>     - CMakeLists.txt
>     - **YOUR ROSCPP PACKAGE**
>       - CMakeLists.txt (This is the one you edit)
>       - package.xml (This too!)
>       - src
>         - **PUT YOUR C++ SCRIPTS HERE**

**3. Write your package description (package.xml)**

For more info, read the **catkin tutorial**

Example package.xml:

```python
<?xml version="1.0"?>
<package format="2">
  <name>basic_pub_sub</name>
  <version>0.0.0</version>
  <description>A minimal roscpp basic pub-sub package!</description>

  <author email="methyldragon@gmail.com">methylDragon</author>
  <maintainer email="methyldragon@gmail.com">methylDragon</maintainer>
  <url type="website">http://github.com/methylDragon</url>

  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>

  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  
</package>
```

**4. Configure the build (CMakeLists.txt)**

For more info, read the **catkin tutorial**

Example CMakeLists.txt:

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(basic_pub_sub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# Initialise the export variables
catkin_package(
  INCLUDE_DIRS src
  # LIBRARIES blah
  CATKIN_DEPENDS roscpp std_msgs 
  DEPENDS system_lib
)

# THIS IS SUPER IMPORTANT!
# You need this to let catkin find ROS
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# One per executable!
add_executable(basic_pub src/basic_pub.cpp)
target_link_libraries(basic_pub ${catkin_LIBRARIES})

add_executable(basic_sub src/basic_sub.cpp)
target_link_libraries(basic_sub ${catkin_LIBRARIES})

add_executable(hello_world src/hello_world.cpp)
target_link_libraries(hello_world ${catkin_LIBRARIES})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

**6. Build the package**

- Go to the root of your workspace
- Run `catkin_make` and ensure no errors occured
- Source your workspace again `source devel/setup.bash`

**7. Verify the package**

Start a ROS master

```shell
$ roscore
```

Run your nodes

```shell
$ rosrun your_package_name node_name
```



#### **False Failures**

> You might find that the first time you run your package, the command will not autocomplete, because ROS takes awhile to find the package.
>
> To speed it along, either manually type out the rosrun command, or use `rospack profile` to rebuild the package tree.



### 2.16 Extra roscpp tips  <a name="2.16"></a>

[go to top](#top)

#### **Modularisation and OOP**

Adapted from: https://github.com/ethz-asl/ros_best_practices

---

**Pre-Requisites:**

- Proficient in C++ OOP (read the C++ crash course OOP section!)
- This is a wiiiild ride...

Read the rospy modularisation section first to get an idea of what's going on!

---

![2.2](images/2.2.png)

Source: ETHz

>  **Example folder structure:**
>
> - my_catkin_ws
>   - CMakeLists.txt
>   - src
>     - **YOUR_ROSCPP_PACKAGE**
>       - CMakeLists.txt
>       - package.xml
>       - config
>         - default.yaml
>       - launch
>         - launch_Node.launch
>       - include
>         - **PACKAGE_HEADERS**
>           - algorithm.hpp
>           - nodeClassInterface.hpp
>       - src
>         - algorithm.cpp
>         - nodeClassInterface.cpp
>         - Node.cpp
>
> Note: Names files in the package /launch,  /src, and /include don't matter as long as you import the stuff properly

Where:

>  Since the code is way too long due to all the header files and imports and stuff, refer to the GitHub source link or the minimal project for examples

- **Node.cpp** (Starts the node)

  

- **nodeClassInterface** (Node interface)

  - .hpp (header file) (Interface)
  - .cpp (source code) (Implementation)

- **algorithm** (Algorithmic part of the node) (Can be separated into a ROS-independent library) 

  - .hpp (header file) (Interface)

  - .cpp (source code) (Implementation)

    

- **default.yaml** (For loading parameters)

- **launch_Node.launch** (For loading parameters on node start)

  

- **package.xml** (Good old 'friends')

- **CMakeLists.txt** (Make sure you configure this properly)




```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\
```

