#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Operation.hpp>
#include <rtt/Property.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include <orca/orca.h>

namespace rtt_orca
{
namespace util
{
    class RttOrcaRegister: public RTT::Service
    {
    public:
        RttOrcaRegister(RTT::TaskContext* owner)
        : RTT::Service("orca_helper",owner)
        , owner_(owner)
        {
            if(owner_)
            {
                RTT::log(RTT::Info) << "RttOrcaRegister created for task " << owner->getName() << RTT::endlog();
            }
            this->addOperation("activateAll", &RttOrcaRegister::activateAll, this, RTT::ClientThread);
            this->addOperation("desactivateAll", &RttOrcaRegister::desactivateAll, this, RTT::ClientThread);
            this->addOperation("insertAllInProblem", &RttOrcaRegister::insertAllInProblem, this, RTT::ClientThread);
            this->addOperation("removeAllFromProblem", &RttOrcaRegister::removeAllFromProblem, this, RTT::ClientThread);
            this->addOperation("printAll", &RttOrcaRegister::printAll, this, RTT::ClientThread);
        }
        
        void activateAll()
        {
            for(auto t : orca::optim::OptimisationVector().getAllCommons())
            {
                RTT::log(RTT::Info) << "Activating " << t->getName() << RTT::endlog();
                t->activate();
            }
        }
        
        void desactivateAll()
        {
            for(auto t : orca::optim::OptimisationVector().getAllCommons())
            {
                RTT::log(RTT::Info) << "Desactivating " << t->getName() << RTT::endlog();
                t->desactivate();
            }
        }
        
        void insertAllInProblem()
        {
            for(auto t : orca::optim::OptimisationVector().getAllCreatedObjects())
            {
                RTT::log(RTT::Info) << "Inserting " << t->getName() <<  " in problem" << RTT::endlog();
                t->insertInProblem();
            }
        }
        
        void removeAllFromProblem()
        {
            if(!getOwner())
            {
                return;
            }

            for(auto pname : getOwner()->getPeerList())
            {
                if( getOwner()->getPeer(pname)->provides()->hasOperation("removeFromProblem") )
                {
                    RTT::log(RTT::Info) << "Removing peer " << pname << " from problem"  << RTT::endlog();
                    RTT::OperationCaller<bool()> peer_function(getOwner()->getPeer(pname)->getOperation("removeFromProblem"), getOwner()->getPeer(pname)->engine());
                    if (!peer_function.ready()) continue ;
                    if(peer_function())
                    {
                        RTT::log(RTT::Info) << "Peer " << pname << " successfully removed from problem"  << RTT::endlog();
                    }
                }
            }
        }
        
        void printAll()
        {
            for(auto t : orca::optim::OptimisationVector().getAllCommons())
            {
                RTT::log(RTT::Info) << "Printing " << t->getName() << " in problem"  << RTT::endlog();
                t->print();
            }
        }
    private:
        RTT::TaskContext * owner_;
    };
}
}


ORO_SERVICE_NAMED_PLUGIN(rtt_orca::util::RttOrcaRegister,"orca_helper")
