## Catkin Reference

Author: methylDragon  
Reference notes for the CMake-based build system for ROS! I'm using ROS Kinetic at the moment, but these notes should apply for any ROS version Indigo onwards!  
Most of the notes will be adapted from: http://wiki.ros.org/catkin/Tutorials

---

## Pre-Requisites

- You need to have Catkin installed! Luckily it comes pre-installed with ROS
  - Otherwise... http://wiki.ros.org/catkin/Tutorials/create_a_workspace
- !!! **I'll presume you know your way around the Linux Terminal** !!!
  - If not... play http://web.mit.edu/mprat/Public/web/Terminus/Web/main.html for a start!
- Make sure you've sourced your ROS environment!
  - `source /opt/ros/<DISTRO>/setup.bash`
  - In the case of Kinetic, `source /opt/ros/kinetic/setup.bash`
    - It would be good to append said line to your .bashrc file! Access said file using `sudo nano ~/.bashrc`



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)  
2. [Basic Catkin Workflow](#2)    
   2.1   [Workflow Introduction](#2.1)    
   2.2   [Making a Workspace](#2.2)    
   2.3   [Sourcing the Workspace](#2.3)    
   2.4   [Catkin Packages](#2.4)    
   2.5   [Creating a catkin Package](#2.5)    
   2.6   [Building the Workspace](#2.6)    
   2.7   [More about catkin_make](#2.7)    
   2.8   [Installing Dependencies](#2.8)    
3. [Customising your packages: Package.xml](#3)    
   3.1   [package.xml](#3.1)    
   3.2   [Example package.xml](#3.2)    
   3.3   [Description](#3.3)    
   3.4   [Maintainers](#3.4)    
   3.5   [License](#3.5)    
   3.6   [Author Info](#3.6)    
   3.7   [Dependencies](#3.7)    
4. [Customising your packages: CMakeLists.txt](#4)    
   4.1   [CMakeLists.txt](#4.1)    
   4.2   [Required CMake Version](#4.2)    
   4.3   [Package Name](#4.3)    
   4.4   [Declare catkin Dependencies](#4.4)    
   4.5   [Enable Python module support](#4.5)    
   4.6   [Add Messages, Services, and Actions](#4.6)    
   4.7   [Generate Messages, Services, and Actions](#4.7)    
   4.8   [Specify Package Build-info](#4.8)    
   4.9   [Declare Other Dependencies](#4.9)    
   4.10 [Specify Build Targets (and more!)](#4.10)    
   4.11 [Add Unit Test Handlers (Optional)](#4.11)    
   4.12 [Specify Installable Targets (and Python Scripts!)](#4.12)    
   4.13 [Use a Custom Compiler Version)](#4.13)    
   

## 1. Introduction

Long story short, when you're making packages for a huge ecosystem such as ROS, you're bound to end up with a lot of problems managing nested dependencies in its many packages.

Catkin is the next generation of the original answer to that problem (rosbuild.) It helps you manage dependencies, manage workspaces (environments), build ROS packages, and more!



## 2. Basic Workflow <a name="2"></a>

### 2.1 Workflow Introduction <a name="2.1"></a>

[go to top](#top)

Now that we're ready to go, let's get some context!

The steps you'll generally go through when developing for ROS will go roughly as such

1. Make a catkin workspace (and overlaying it into your environment)
   - `catkin_make`
2. Create catkin packages (ROS packages)
   - `catkin_create_pkg`
3. Build the catkin workspace and its packages (and overlay them into your environment)
   - `catkin_make` again
4. Install the catkin workspace and its packages
   - `catkin_make install`



### 2.2 Making a Workspace <a name="2.2"></a>

[go to top](#top)

Before we can do anything with Catkin, we need to make a workspace for it. A workspace is a handy way to partition all your required packages, perfect for neatly packaging individual projects together!

Ok. So if you want to make a workspace, navigate to whatever directory you want to make it in using `cd`. Then,

```shell
$ mkdir -p catkin_ws/src # Create a catkin_ws folder with a src folder in it
$ cd catkin_ws # Navigate into the folder's root
$ catkin_make # Build the required files
```



### 2.3 Sourcing the workspace <a name="2.3"></a>

[go to top](#top)

Great! Now **source** into it to allow you to run commands from the workspace in your Terminal. (You're overlaying the workspace's commands into your environment.)

```shell
$ source devel/setup.bash # Make sure you run this from your workspace's root!

# It would be good to also append this source command to your .bashrc so you don't have to keep doing it everytime you open a terminal

# Append source <workspace_directory>/devel/setup.bash
```

To check if you've sourced into it properly, run

```shell
$ echo $ROS_PACKAGE_PATH

# Ensure your workspace's directory appears in here
```



### 2.4 Catkin Packages <a name="2.4"></a>

[go to top](#top)

**ROS packages are catkin packages!**

Every catkin package requires you to have the following files and folders:

- package.xml
  - Provides metadata about the package (maintainer, description, etc.)
- CMakeLists.txt
  - Describes how to build the code and where to install it
  - http://wiki.ros.org/catkin/CMakeLists.txt
- The individual package folders

The simplest package folder structure structure looks like this:

```
my_package/
  CMakeLists.txt
  package.xml
```

So the corresponding workspace folder will look like this

```
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
```



### 2.5 Creating a catkin Package <a name="2.5"></a>

[go to top](#top)

Go into the **src** directory of the catkin workspace you want to create a package in

```shell
$ cd <workspace_directory>/catkin_ws/src
```

Run the **catkin_create_pkg** script to create your package! The command works like this

```shell
$ catkin_create_package <package_name> [depend1] [depend2] ...
```

This will create a package folder with its respective package.xml and CMakeLists.txt, filled somewhat with the information you gave the script.



#### **Finding Package Dependencies via rospack**

First-order package dependencies are stored in the package.xml file.

You can also see them using

```shell
$ rospack depends1 <package_name>
```

If you want to see ALL the nested dependencies, use

```shell
$ rospack depends <package_name>
```

> Note: You can make catkin_make ignore your package by leaving an empty file with the name **CATKIN_IGNORE** in the package's directory!



### 2.6 Building the Workspace <a name="2.6"></a>

[go to top](#top)

Now all you need to do is build the packages in your workspace!

```shell
$ cd <workspace_directory>/catkin_ws
$ catkin_make
```

catkin_make will now do certain things to certain files:

- **Packages in the source space (src)** will be built into the **build space (/build)**
- **Source files, scripts, and other static files** will remain in the **source space (/src)** 
- **Generated files (libraries, executables, etc.)** will be placed in the **devel space (/devel)**



#### **Devel and Install spaces**

> CHOOSE BETWEEN USING THE INSTALL OR DEVEL SPACE
>
> DON'T USE MORE THAN ONE

The development space (devel) is useful for, guess what, developing. When you're using `catkin_make` you don't need to invoke the `install` argument each time.

The install space (install) is there for when you're ready to distribute.



#### **Sourcing the built workspace**

You know the drill. Find the setup.bash, and source it. Appending it to .bashrc will be useful.

```shell
$ source <workspace_directory>/<devel_or_install>/setup.bash
```



### 2.7 More about catkin_make <a name="2.7"></a>

[go to top](#top)

> This section will talk about customising the behaviour of catkin_make!
>
> Though, there's another tool that's helpful if you want to trudge through the documentation for that! I'm just going to be (mostly) writing for the official catkin_make tool though! The extra tool I'm talking about is here: http://catkin-tools.readthedocs.io/en/latest/index.html

Sometimes your build might fail at a certain package (eg, at package 100 out of 150).

If you just ran catkin_make again, it'll rebuild everything starting from package 1. How do you fix stuff like that? Turns out there are more parameters you can use for catkin_make! (Actually you can customise it even more by changing your CMakeLists.txt, but if you don't want to do that, there's a lot you can do from the terminal!)



#### **Build specific packages**

```shell
$ catkin_make --pkg <package A> <package B>
```



#### **Build starting from a specific package**

```shell
$ catkin_make --from-pkg <package>
```



#### **Previewing the build order (catkin tools preview!)**

Now we'll talk about a handy command we can use from the third-party catkin tools!: http://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html

This command lets you preview the build order. Very useful for saving time if you know what you want to build or where you had an error!

```shell
$ catkin build --dry-run
```



### 2.8 Installing Dependencies <a name="2.8"></a>

[go to top](#top)

Sometimes you might find that you don't have dependencies installed from a package you downloaded online. No issues! Use **rosdep** to install all the dependences!

```shell
# I'm assuming you're going to be using ROS Kinetic

# Navigate to your workspace's root and run:
$ rosdep install —from-paths src —ignore-src —rosdistro=kinetic -y
```



## 3. Customising your packages: Package.xml <a name ="3"></a>

### 3.1 package.xml <a name="3.1"></a>

[go to top](#top)

package.xml contains metadata about your package! It's written in **XML**, so it shouldn't be that hard to understand.

It contains the following data: (Order matters! Play it safe!)

- Description
- Maintainers
- License
- Author Info
- Dependencies
- ...



### 3.2 Example package.xml <a name="3.2"></a>

[go to top](#top)

From the catkin tutorials

```xml
<?xml version="1.0"?>
<package format="2">
  <name>beginner_tutorials</name>
  <version>0.1.0</version>
  <description>The beginner_tutorials package</description>

  <maintainer email="you@yourdomain.tld">Your Name</maintainer>
  <license>BSD</license>
  <url type="website">http://wiki.ros.org/beginner_tutorials</url>
  <author email="you@yourdomain.tld">Jane Doe</author>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

</package>

```



### 3.3 Description <a name="3.3"></a>

[go to top](#top)

The description tag can be anything you'd like. But the convention is that the very first sentence should be able to summarise the point of the package.

```xml
<description>The beginner_tutorials package</description>
```



### 3.4 Maintainers <a name="3.4"></a>

[go to top](#top)

You can mention more than one maintainer. One per line. Make sure you have the email and the name!

```xml
 <maintainer email="methylDragon@gmail.com">methylDragon</maintainer>
```



### 3.5 License <a name="3.5"></a>

[go to top](#top)

Licenses are required, one per line also.

Common licenses are: BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3.

Don't know what license to use? No worries! https://choosealicense.com/

```xml
  <license>MIT</license>
```



### 3.6 Author Info <a name="3.6"></a>

[go to top](#top)

Not required, but handy

```xml
  <url type="website">http://github.com/methylDragon</url>
  <author email="methylDragon@gmail.com">methylDragon</author>
```



### 3.7 Dependencies <a name="3.7"></a>

[go to top](#top)

REQUIRED. Arguably the most important of them all.

State your dependencies here! For either catkin or system dependencies, there are a couple of ways to state them:

**For packages you need at compile time**

```xml
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
```

**For build tool packages**

```xml
  <buildtool_depend>catkin</buildtool_depend>
```

**For dependencies you need at runtime**

```xml
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```

**For dependencies you need for testing only**

```xml
  <test_depend>roscpp</exec_depend>
  <test_depend>rospy</exec_depend>
  <test_depend>std_msgs</exec_depend>
```



## 4. Customising your packages: CMakeLists.txt <a name="4"></a>

### 4.1 CMakeLists.txt <a name="4.1"></a>

[go to top](#top)

CMakeLists.txt is what gets passed into the CMake build system when you run catkin_make. It describes where and how to build the code, and where to install the code to.

> Your CMakeLists.txt file **MUST** follow this format otherwise your packages will not build correctly. The order in the configuration **DOES** count. 
>
> 1. **Required CMake Version** (`cmake_minimum_required`) 
> 2. **Package Name** (`project()`) 
> 3. **Declare catkin Dependencies** (`find_package()`) 
> 4. **Enable Python module support** (`catkin_python_setup()`) 
> 5. **Add Messages, Services, and Actions** (`add_message_files(), add_service_files(), add_action_files()`) 
> 6. **Generate Messages, Services, and Actions** (`generate_messages()`) 
> 7. **Specify Package Build-Info** (`catkin_package()`) 
> 8. **Declare other dependencies** (`add_executable()`, `add_dependencies()`, `add_action_files()`)
> 9. **Specify Build Targets (and more!)**(`add_library()/add_executable()/target_link_libraries()`) 
> 10. **Add Unit Test Handlers (Optional) ** (`catkin_add_gtest()`) 
> 11. **Specify Installable Targets** (`install()`) 
>
> (http://wiki.ros.org/catkin/CMakeLists.txt)



> **Handy tip:**
>
> If you want to set 'variables' for the names or paths of install targets, use `set`
>
> ```python
> # Set the name to use for the executable.
> set (BINNAME1 pid_configure)
> set (BINNAME2 pid_listen)
>
> # Set the source files to use with the executable.
> set (SRCS1 ${SRCS1} src/lino_pid_core.cpp)
> set (SRCS1 ${SRCS1} src/pid_configure.cpp)
> set (SRCS2 ${SRCS2} src/lino_pid_core.cpp)
> set (SRCS2 ${SRCS2} src/pid_listen.cpp)
>
> # Build the executable that will be used to run this node.
> add_executable (${BINNAME1} ${SRCS1})
> target_link_libraries(${BINNAME1} ${catkin_LIBRARIES})
> add_dependencies(${BINNAME1} ${PROJECT_NAME}_gencfg)
> add_executable (${BINNAME2} ${SRCS2})
> target_link_libraries(${BINNAME2} ${catkin_LIBRARIES})
> add_dependencies(${BINNAME2} ${PROJECT_NAME}_gencfg)
> ```
>
> 



### 4.2 Required CMake Version (`cmake_minimum_required`) <a name="4.2"></a>

[go to top](#top)

Requires version 2.8.3 or higher

```python
cmake_minimum_required(VERSION 2.8.3)
```



### 4.3 Package Name (`project()`) <a name="4.3"></a>

[go to top](#top)

Self-Explanatory

```python
project(package_name)
```



### 4.4 Declare catkin Dependencies (`find_package()`) <a name="4.4"></a>

[go to top](#top)

This is where you state what other CMake packages you need to use to build your project.

There is always at least this dependency though! (Since you require catkin in order to build your project)

```python
# This is if you have no dependencies other than catkin
find_package(catkin REQUIRED)

# But if you have more...
# State them as components!
find_package(catkin REQUIRED COMPONENTS package1 package2 package3)
```

find_package() creates some environment variables once catkin finds the package. These variables tell CMake where the relevant components of the package are (header files, source, libraries, dependencies, paths, etc.)

> <NAME>_FOUND - Set to true if the library is found, otherwise false 
>
> <NAME>_INCLUDE_DIRS or <NAME>\_INCLUDES - The include paths exported by the package 
>
> <NAME>_LIBRARIES or <NAME>\_LIBS - The libraries exported by the package 
>
> <NAME>_DEFINITIONS - ? 

If you don't state the component as a component, the paths, libraries, etc. stated in the quoted block above won't get added. So please **add them as components**.



### 4.5 Enable Python module support (`catkin_python_setup()`) <a name="4.5"></a>

[go to top](#top)

Use this if your ROS package uses python.

Create a setup.py file, and add

```python
catkin_python_setup()
```

**DO NOT USE THIS ALONE TO INSTALL PYTHON SCRIPTS YOU WANT TO USE AS A PACKAGE IN THE INSTALL SPACE.** There's a section for that later on!



### 4.6 Add Messages, Services, and Actions (`add_message_files(), add_service_files(), add_action_files()`)  <a name="4.6"></a>

[go to top](#top)

Messages, Services, and Actions files are hugely important in ROS. Catkin has to prepare them before they can be used by ROS though!

So add them!

```python
# Declare the message files to be built
  add_message_files(FILES
    MyMessage1.msg
    MyMessage2.msg
  )

  # Declare the service files to be built
  add_service_files(FILES
    MyService.srv
  )
    
  # Declare the action files to be built
  add_action_files( FILES
    File1
    File2
  )
```



### 4.7 Generate Messages, Services, and Actions  (`generate_messages()`)  <a name="4.7"></a> 

[go to top](#top)

Simple enough

```python
generate_messages()
```



### 4.8 Specify Package Build-Info (`catkin_package()`, `include_directories`)  <a name="4.8"></a> 

[go to top](#top)

(OPTIONAL) But you **MUST** have this included before you can add_library() or add_executable()

This is if you need to import modules

> CATKIN_DEPENDS are catkin packages
>
> DEPENDS are system dependencies
>
> $(PROJECT_NAME) evaluates to whatever you passed project()

```python
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp nodelet
   DEPENDS eigen opencv)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(~/some_lib_dirs)
```

`include_directories`:  For C++ library paths (since you include them)

`link_directories`: Used to add additional library paths (not recommended)



### 4.9 Declare Other Dependencies (`add_executable()`, `add_dependencies()`, `add_action_files()`) <a name="4.9"></a>

[go to top](#top)

```python
  # define executable to be built using messages etc.
  # and the dependencies it needs
  add_executable(message_program src/main.cpp)
  add_dependencies(message_program ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  # define executable not using any messages/services provided by this package
  # and the dependencies it needs
  add_executable(does_not_use_local_messages_program src/main.cpp)
  add_dependencies(does_not_use_local_messages_program ${catkin_EXPORTED_TARGETS})
```



### 4.10 Specify Build Targets (and more!) (`add_library(), add_executable(), target_link_libraries()`) <a name="4.10"></a> 

[go to top](#top)

There are generally two ways to specify a build target

- Executable: Something you run
- Library: Something that is used by executable targets



#### **Name your targets!** They have to be unique! You can rename stuff though!

```python
set_target_properties(rviz_image_view
                      PROPERTIES OUTPUT_NAME image_view
                      PREFIX "")
```



#### **Specify a custom output directory**

Sometimes you might want a different directory to install to for certain packages (like python module installs)

```python
set_target_properties(python_module_library
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
```



#### **Include Paths and Library Paths**

You need to specify the locations of your resources before specifying your targets

- Include paths: Header files, and the like
- Library paths: Libraries, of course

```python
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
```



#### **Specify your targets**

Executable target

```python
# This will build a program called myProgram from the three source files stated
add_executable(myProgram src/main.cpp src/some_file.cpp src/another_file.cpp)
```

Library Target

```python
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
```



#### **Link Libraries**

Then you link your libraries to your executables!

```python
target_link_libraries(<executableTargetName> <lib1> <lib2> ... <libN>)
```



### 4.11 Add Unit Test Handlers (Optional)  (`catkin_add_gtest()`)  <a name="4.11"></a>

[go to top](#top)

```python
catkin_add_gtest(myUnitTest test/utest.cpp)
```



### 4.12 Specify Installable Targets (and Python Scripts!) (`install()`, `catkin_install_python()`) <a name="4.12"></a> 

[go to top](#top)

> Make sure catkin_package() is called BEFORE install()!

Normally, `catkin_make` places built targets into the **devel space (/devel)**. But if you want to do a proper release where users build using `catkin_make install`, use install() to place built targets into the **install space (/install)**.

install() takes the following arguments:

- `TARGETS` - which targets to install 
- `ARCHIVE DESTINATION` - Static libraries and DLL (Windows) .lib stubs 
- `LIBRARY DESTINATION` - Non-DLL shared libraries and modules 
- `RUNTIME DESTINATION` - Executable targets and DLL (Windows) style shared libraries 

**For example:**

```python
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

**Or if you want to install to a specific folder:** (In this case, a Python library folder)

```python
install(TARGETS python_module_library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
```

**Or if you want to install python scripts**

```python
catkin_install_python(PROGRAMS scripts/myscript
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```

**Or if you want to install header files**

```python
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
```

**Or if you want to install roslaunch files, or other resources**

```python
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
```

​    

### 4.13 Use a Custom Compiler Version <a name="4.13"></a>

[go to top](#top)

Sometimes you might want ROS to compile with a different compiler version to leverage certain features of that compiler.

This one sets the compiler to use to C++11! (Which allows one to use list initialisation! Super useful for making maps.)

```python
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
```



------

[![Yeah! Buy the DRAGON a COFFEE!](../../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)