
#ifndef _DDSENTITYMGR_
  #define _DDSENTITYMGR_


  #include "ccpp_dds_dcps.h"
  #include "CheckStatus.h"
  using namespace DDS;

  class DDSEntityManager
  {

      /* Generic DDS entities */
      DomainParticipantFactory_var dpf;
      DomainParticipant_var participant;
      Topic_var topic;
      Publisher_var publisher;
      Subscriber_var subscriber;
      DataWriter_var writer;
      DataReader_var reader;

      /* QosPolicy holders */
      TopicQos reliable_topic_qos;
      TopicQos setting_topic_qos;
      PublisherQos pub_qos;
      DataWriterQos dw_qos;
      SubscriberQos sub_qos;

      DomainId_t domain;
      ReturnCode_t status;

      DDS::String_var partition;
      DDS::String_var typeName;
    public:
      void createParticipant(const char *partitiontName);
      void deleteParticipant();
      void registerType(TypeSupport *ts);
      void createTopic(char *topicName);
      void deleteTopic();
      void createPublisher();
      void deletePublisher();
      void createWriter();
	  void createWriter(bool autodispose_unregistered_instances);
	  void deleteWriter();
      void createSubscriber();
      void deleteSubscriber();
      void createReader();
      void deleteReader();
      DataReader_ptr getReader();
      DataWriter_ptr getWriter();
      Publisher_ptr getPublisher();
      Subscriber_ptr getSubscriber();
      Topic_ptr getTopic();
      DomainParticipant_ptr getParticipant();
      ~DDSEntityManager();
  };

#endif
