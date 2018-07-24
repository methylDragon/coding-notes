#include "Participant.h"

Participant::Participant(const char * partitiontName , DomainId_t domain_id)
{
    //TODO: pass domain_id arg to domain var;
    domain = DOMAIN_ID_DEFAULT;
    dpf = DomainParticipantFactory::get_instance();
    checkHandle(dpf.in(), "DDS::DomainParticipantFactory::get_instance");
    participant = dpf->create_participant(domain, PARTICIPANT_QOS_DEFAULT, NULL,
      STATUS_MASK_NONE);
    checkHandle(participant.in(),
      "DDS::DomainParticipantFactory::create_participant");
    partition = partitiontName;
}

DDS::String_var Participant::get_partition()
{
    return partition;
}

DomainParticipant_var Participant::get_participant()
{
    return participant;
}

void Participant::kill()
{
  status = dpf->delete_participant(participant.in());
  checkStatus(status, "DDS::DomainParticipant::delete_participant");
}
