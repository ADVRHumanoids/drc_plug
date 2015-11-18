/* Copyright [2014,2015] [Corrado Pavan, Alessandro Settimi, Luca Muratore]
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

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

