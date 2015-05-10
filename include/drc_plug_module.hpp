#ifndef drc_plug_MODULE_HPP_
#define drc_plug_MODULE_HPP_

#include <GYM/control_module.hpp>

#include "drc_plug_thread.h"
#include "drc_plug_constants.h"

/**
 * @brief drc_plug module derived from control_module
 * 
 * @author 
 */

using namespace walkman::drc::plug;

class drc_plug_module : public control_module<drc_plug_thread> {
public:
    
    /**
     * @brief constructor: do nothing but construct the superclass
     * 
     */
    drc_plug_module(    int argc, 
                               char* argv[],
                               std::string module_prefix, 
                               int module_period, 
                               yarp::os::ResourceFinder rf ) : control_module<drc_plug_thread>(  argc, 
                                                                                            		argv, 
                                                                                            		module_prefix, 
                                                                                            		module_period,
                                                                                            		rf )
    {
    }
    
    /**
     * @brief overriden function to specify the custom params for the param helper
     * 
     * @return a vector of the custom params for the param helper
     */
    virtual std::vector< paramHelp::ParamProxyInterface* > custom_get_ph_parameters() 
    {
	// TODO: function skeleton
        std::vector<paramHelp::ParamProxyInterface *> custom_params;
        return custom_params;
    }
    
    
};

#endif
