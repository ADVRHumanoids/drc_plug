#include <yarp/os/all.h>
#include <GYM/control_module.hpp>
#include <cstdlib>

#include "drc_plug_module.hpp"

// default module period
#define MODULE_PERIOD 1000 //[millisec]

int main(int argc, char* argv[])
{
    // yarp network declaration and check
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
        std::cerr <<"yarpserver not running - run yarpserver"<< std::endl;
        exit(EXIT_FAILURE);
    }
    // yarp network initialization
    yarp.init();

    // create rf
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile( "bigman_drc_plug.ini" ); 
    rf.setDefaultContext( "drc_plug" );  
    rf.configure(argc, argv);
    // create my module
    drc_plug_module drc_plug_mod = drc_plug_module( argc, argv, "drc_plug", MODULE_PERIOD, rf );
    
    // run the module
    drc_plug_mod.runModule( rf );
    
    // yarp network deinitialization
    yarp.fini();
    
    exit(EXIT_SUCCESS);
}
