#ifndef drc_plug_THREAD_H_
#define drc_plug_THREAD_H_

#include <GYM/control_thread.hpp>

/**
 * @brief drc_plug control thread
 * 
 **/
class drc_plug_thread : public control_thread
{
private:   
    
public:
    
    /**
     * @brief constructor
     * 
     * @param module_prefix the prefix of the module
     * @param rf resource finderce
     * @param ph param helper
     */
     drc_plug_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr<paramHelp::ParamHelperServer> ph );
    
    
    /**
     * @brief drc_plug control thread initialization
     * 
     * @return true on succes, false otherwise
     */
    virtual bool custom_init();
    
    /**
     * @brief drc_plug control thread main loop
     * 
     */
    virtual void run();
    
};

#endif
