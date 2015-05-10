#ifndef PLUG_STACK_H_
#define PLUG_STACK_H_

#include <idynutils/idynutils.h>
#include <OpenSoT/OpenSoT.h>
#include <OpenSoT/utils/AutoStack.h>

namespace walkman { namespace drc { namespace plug {
    class plug_stack : public OpenSoT::AutoStack {
    public:
        typedef boost::shared_ptr<plug_stack> Ptr;

        OpenSoT::tasks::velocity::Cartesian::Ptr left_arm_task;
        OpenSoT::tasks::velocity::Cartesian::Ptr right_arm_task;
        OpenSoT::tasks::velocity::Cartesian::Ptr right_foot_task;
        OpenSoT::tasks::velocity::Cartesian::Ptr pelvis_task;
        OpenSoT::tasks::velocity::Postural::Ptr postural;
        OpenSoT::tasks::velocity::CoM::Ptr com_task;

        plug_stack(const double dT,
                    iDynUtils& model,
                    const yarp::sig::Vector& q);
    };

} } }

#endif

