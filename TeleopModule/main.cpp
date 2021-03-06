#include <iostream>
#include <stdlib.h>
#include <qi/os.hpp>

#include <boost/shared_ptr.hpp>

#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>

#include "TeleopModule.h"

#ifndef _WIN32
# include <signal.h>
#endif

#ifdef TELEOPMODULE_IS_REMOTE
# define ALCALL
#else
// when not remote, we're in a dll, so export the entry point
# ifdef _WIN32
#  define ALCALL __declspec(dllexport)l
# else
#  define ALCALL
# endif
#endif


extern "C"
{
ALCALL int _createModule(boost::shared_ptr<AL::ALBroker> broker)
{
    // init broker with the main broker instance
    // from the parent executable
    AL::ALBrokerManager::setInstance(broker->fBrokerManager.lock());
    AL::ALBrokerManager::getInstance()->addBroker(broker);
    // create module instances
    AL::ALModule::createModule<TeleopModule>(broker, "TeleopModule");
    return 0;
}

ALCALL int _closeModule(  )
{
    return 0;
}
} // extern "C"

#ifdef TELEOPMODULE_IS_REMOTE
  int main(int argc, char *argv[])
  {
    // pointer to createModule
    TMainType sig;
    sig = &_createModule;
    // call main
    return ALTools::mainFunction("TeleopModule", argc, argv, sig);
  }
#endif
