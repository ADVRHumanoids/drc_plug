#include <yarp/os/all.h>

#include "drc_plug_thread.h"
#include "drc_plug_constants.h"

drc_plug_thread::drc_plug_thread( std::string module_prefix, 
                             			yarp::os::ResourceFinder rf, 
                             			std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    control_thread( module_prefix, rf, ph )
{
    // TODO: skeleton constructor
}

bool drc_plug_thread::custom_init()
{
    // TODO: skeleton function   
    return true;
}

void drc_plug_thread::run()
{   
    // TODO: skeleton function
}    
