#ifndef BOILERPLATE_H
#define BOILERPLATE_H

#include <iostream>
#include "ccpp_dds_dcps.h"
#include "CheckStatus.h"
#include "Participant.h"
#include "ccpp_Boilerplate.h"

using namespace DDS;
namespace BoilerplateMsg
{
    class Publisher
    {
        void init_publisher();
        
        DomainParticipant_var participant;
        ReturnCode_t status;
        
        String_var partition;   
        char * topicName;
        
        TypeSupport * ts;   
        DDS::String_var typeName;
        
        Topic_var topic;
        TopicQos reliable_topic_qos;
        Publisher_var publisher;
        PublisherQos pub_qos;
        
        DataWriterQos dw_qos;
        DataWriter_var writer;
        TemplateDataWriter_var myWriter;

        public:
            Publisher(Participant  &, char * );
            void publish(Template);
            void kill();
    };

    class Subscriber
    {
        void init_subscriber();
        
        DomainParticipant_var participant;
        ReturnCode_t status;
        
        String_var partition;    
        char * topicName;

        TypeSupport * ts;   
        DDS::String_var typeName;
        
        Topic_var topic;
        TopicQos reliable_topic_qos;
        Subscriber_var subscriber;
        SubscriberQos sub_qos;
        
        DataReader_var reader;
        TemplateDataReader_var myReader;

        public:
            Subscriber(Participant  &, char * );
            TemplateSeq msg_list;
            void read();
            void kill();
    };

}


#endif 
