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
2. [OpenSplice DDS In A Nutshell](#2)    
   2.1   [Architecture](#2.1)    
   2.2   [Domain Participants](#2.2)    
   2.3   [Topics](#2.3)    
   2.4   [A Basic DDS Application](#2.4)    
   2.5   [Reading and Writing Data (Alternate Example)](#2.5)    




## 1. Introduction <a name="1"></a>

So what's DDS exactly?

DDS stands for **Data Distribution Service** (for real-time systems.) DDS is used very widely nowadays, with applications in Defense and Aerospace, Commercial sectors, etc. And is recommended by key administration worldwide!

So think **ROS** and Publisher/Subscribers as opposed to monolithic server-client architectures like stuff using the LAMP stack (the standard web service stuff.) This allows for more fault resilient network architectures that support decentralisation and decoupling (both spatially and temporally) of processing and data instead of hard-coded relations like in the standard server-client model.

**OpenSplice DDS** is one of the implementations of the OMG (Object Management Group) DDS standards. It offers Pub/Sub functionality as well as Quality Of Service (QoS) parameters that you can choose (i.e. choosing metaparameters for you data like data longevity, transience, timeliness, etc.)

OpenSplice DDS can be implemented using the Scala programming language, but it has an interface that uses C++, which is what we're going to be using.

> To the question ‘What is DDS?’ one answer is:
> **It is a Pub/Sub technology for ubiquitous, polyglot, efficient and secure data sharing.**
>
> Another way of answering this question is to say that:
> **DDS is Pub/Sub on steroids.**



## 2. OpenSplice DDS In A Nutshell <a name="2"></a>

This is the short version to get started real quick! We'll do more elaborations in a later section.



### 2.1 Architecture <a name="2.1"></a>

[go to top](#top)

I mentioned before DDS is based off of pub-sub architecture.

But specifically, it's a form of Topic-Based Pub-Sub abstraction that includes

- Topics: The subject of data distribution (think ROS!)
- DataWriters: The writers/producers of data
- DataReaders: The consumers of data

DDS matches DataWriters and DataReaders automatically using its Dynamic Discovery service, and the behaviour of the system allows for tweaking of QoS parameters.

Publishers and Subscribers are free to join or leave the cloud (called the Global Data Space (GDS)) anytime they want, since they're dynamically discovered (or locally defined.)

Additionally, the GDS is, unlike in ROS, decentralised, so there's literally no single point of failure, even with individual applications crashing. The system as a whole will carry on.



### 2.2 Domain Participants <a name="2.2"></a>

[go to top](#top)

**Domain Participants** grant you access to the **Global Data Space** (GDS) also called the **domain** in DDS.

Domain IDs are defined by **integers**

```c++
// create a Domain Participant, -1 defaults to value defined in configuration file
dds::domain::DomainParticipant dp(-1);
```



### 2.3 Topics <a name="2.3"></a>

[go to top](#top)

Topics in DDS define **classes of streams**

They have the following properties:

- Unique Name
- Type (user-defined)
- QoS Policies

Example topic definition (**this should go in a header file!**):

```c++
// C++ refresher: Structs are like classes, but the elements are public by default
// People tend to use Structs to just store plain-old-data types (POD) as opposed to methods

struct ShapeType{
    // (Here we're defining color as the topic key! It should preferably be an ID number though)
    string color; //@key  
    long x;
    long y;
    long shapesize;
};
```

#### **Creating a topic**

```c++
// Create a topic (we're creating one with the name Circle now)
// Yes you need the < >
dds::topic::Topic<ShapeType> topic(dp, "Circle");
```



The keys of the topic, defines a unique **stream** or instance of data, which is dealt with by DDS (including lifecycle information as per QoS) 

Each DataWriter can write multiple instances/streams of data.



### 2.4 A Basic DDS Application <a name="2.4"></a>

[go to top](#top)

Every DDS application should include the following things:

- Domain Participant
- Topic
- Publishers and Subscribers
- DataWriters and DataReaders

Remember that **DataWriters** and **DataReaders** write and consume data, but you need **Publishers** and **Subscribers** to push and fetch data that's being produced! And all from **topics**! Under a **domain**!

Here's a example from: http://download.prismtech.com/docs/Vortex/pdfs/OpenSplice_DDSTutorial.pdf

#### **Let's make a topic first**

```c++
// TempControl.idl
enum TemperatureScale {
CELSIUS,
FAHRENHEIT,
KELVIN
};

struct TempSensorType {
short id;
float temp;
float hum;
TemperatureScale scale;
};

#pragma keylist TempSensorType id
```

#### **Writing Data in DDS**

Now we can make our Domain Participants, Topics, Publishers, and Data Writers!

```c++
// create a Domain Participant, -1 defaults to value defined in configuration file
// Note: this is equivalent to
// auto dp = dds::domain::DomainParticipant(-1)
dds::domain::DomainParticipant dp(-1);

// Create the topic
// Same with the dp, this is like topic =
dds::topic::Topic<tutorial::TempSensorType> topic(dp, "TTempSensor");

// Create the Publisher and DataWriter
// So on and so forth.. pub = and dw =
dds::pub::Publisher pub(dp);
dds::pub::DataWriter<tutorial::TempSensorType> dw(pub, topic);

// Write the data
tutorial::TempSensorType sensor(1, 26.0F, 70.0F, tutorial::CELSIUS);
dw.write(sensor);

// Write data using streaming operators (same as calling dw.write(...))
dw << tutorial::TempSensorType(2, 26.5F, 74.0F, tutorial::CELSIUS);
```

#### **Reading Data in DDS**

```c++
// create a Domain Participant, -1 defaults to value defined in configuration file
dds::domain::DomainParticipant dp(-1);

// create the Topic
dds::topic::Topic<tutorial::TempSensorType> topic(dp, "TTempSensor");

// create a Subscriber
dds::sub::Subscriber sub(dp);

// create a DataReader
dds::sub::DataReader<tutorial::TempSensorType> dr(sub, topic);

// keep outputting anything read
while (true) {
    auto samples = dr.read();
    
    std::for_each(samples.begin(), samples.end(), [](const dds::sub::Sample<tutorial::TempSensorType>& s) {
    	std::cout << s.data() << std::endl;
    });
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
}
```

You might be wondering what's up with the std::chrono seconds() call. Basically, the .read() function reads immediately (it is non-blocking, google that.)

So we're intentionally introducing a delay to make sure we don't spin the loop too quickly.

Other than read (polling), there are also two other ways to inform the application that data is available though:

- Polling
- Listeners
  - Think of it like an eventHandler in JS. They can be registered with readers for receiving notifications of available data, status changes, and QoS related stuff
- Waitsets
  - Wait for specified events to occur, then trigger a callback



Anyway, notice how the DataReader and DataWriters aren't coupled. How do they find each other? DDS did it for you!

> Noticed how we used dr.read() to read the data?
>
> There's actually another method called .take() that does the same thing, except it deletes it from the local cache so other readers can't access it. Gasp.



### 2.5 Reading and Writing Data (Alternate Example) <a name="2.5"></a>

[go to top](#top)

Alternative Code examples from: https://www.slideshare.net/Angelo.Corsaro/getting-started-with-dds-in-c-java-and-scala

```c++
// Example DDS application declaration

// Specify the domain
// Remember that the domain ID is an INTEGER!
auto dp = DomainParticipant(domainId);

// Create a topic
// Yes, again you need the < >
auto topic = Topic<ShapeType>(dp, "Circle");

// Create a Publisher and Subscriber
auto pub = Publisher(dp);
auto sub = Subscriber(dp);

// Create a DataWriter and DataReader
auto writer = DatWriter<ShapeType>(pub, topic);
auto reader = DataReader<ShapeType>(sub, topic);
```

Now we can write and read some data!

```c++
// Write Data
writer.write(ShapeType("RED", 131, 107, 89));
// Alternate write statement
writer << ShapeType("RED", 131, 107, 89);

// Read new data
auto data = reader.read();
```

> Note:
>
> Again, depending on your namespaces, you might have to use
>
> `dds::domain::` : When declaring domains
>
> `dds::topic::`: When declaring topics
>
> `dds::pub::` : When declaring publishers and DataWriters 
>
> `dds::sub::` : When declaring subscribers and DataReaders



```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\     
```

​        

------

[![Yeah! Buy the DRAGON a COFFEE!](../../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)