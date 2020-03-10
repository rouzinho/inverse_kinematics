#include "CtrlModule.h"

bool CtrlModule::configure(ResourceFinder &rf)
{
    thr=new CtrlThread(CTRL_THREAD_PER);
    if (!thr->start())
    {
        delete thr;
        return false;
    }

    return true;
}

bool CtrlModule::close()
{
    thr->stop();
    delete thr;

    return true;
}

double CtrlModule::getPeriod()    { return 1.0;  }
bool   CtrlModule::updateModule() { return true; }
