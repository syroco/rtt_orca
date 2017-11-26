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
            this->addOperation("activateAll", &RttOrcaRegister::activateAll, this, RTT::ClientThread);
            this->addOperation("desactivateAll", &RttOrcaRegister::desactivateAll, this, RTT::ClientThread);
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
