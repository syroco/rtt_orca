#include <rtt_orca/task/RttGenericTask.h>

namespace rtt_orca
{
namespace task
{
    template<orca::optim::ControlVariable C>
    class RttRegularisationTask: public orca::task::RegularisationTask<C>, public task::RttGenericTask
    {
    public:
        RttRegularisationTask(const std::string& name)
        : task::RttGenericTask(this,this,name)
        {
            // this->provides("EuclidianNorm")->addOperation("setWeight"
            //         , (void (orca::task::RegularisationTask<C>::*)(double)) &orca::task::RegularisationTask<C>::setWeight
            //         ,this,RTT::OwnThread);
            this->provides("EuclidianNorm")->addOperation("setWeight"
                    , & RttRegularisationTask::setEuclidianNormWeightDiagonal
                    ,this,RTT::OwnThread);
        }

        void setEuclidianNormWeightDiagonal(double weight)
        {
            orca::common::MutexLock lock(orca::task::RegularisationTask<C>::mutex);
            orca::task::RegularisationTask<C>::EuclidianNorm().setWeight(weight);
        }

        void updateHook()
        {
            orca::task::RegularisationTask<C>::update();
        }
    };
}
}

ORO_LIST_COMPONENT_TYPE(rtt_orca::task::RttRegularisationTask<orca::optim::ControlVariable::X>)
ORO_LIST_COMPONENT_TYPE(rtt_orca::task::RttRegularisationTask<orca::optim::ControlVariable::GeneralisedAcceleration>)
ORO_LIST_COMPONENT_TYPE(rtt_orca::task::RttRegularisationTask<orca::optim::ControlVariable::GeneralisedTorque>)
ORO_LIST_COMPONENT_TYPE(rtt_orca::task::RttRegularisationTask<orca::optim::ControlVariable::ExternalWrench>)
ORO_CREATE_COMPONENT_LIBRARY()
