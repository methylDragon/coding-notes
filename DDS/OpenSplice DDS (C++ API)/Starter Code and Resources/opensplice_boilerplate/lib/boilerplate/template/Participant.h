#ifndef PARTICIPANT_H
#define PARTICIPANT_H

#include "ccpp_dds_dcps.h"
#include "CheckStatus.h"

using namespace DDS;

class Participant
{
        
    String_var partition;
    DomainId_t domain;
    ReturnCode_t status;    
    DomainParticipantFactory_var dpf;
    DomainParticipant_var participant;
    
    public:
        Participant(const char *, DomainId_t);        
        String_var get_partition();
        DomainParticipant_var get_participant();
        void kill();
};

#endif 