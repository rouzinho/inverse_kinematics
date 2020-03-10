#ifndef CTRL_MODULE
#define CTRL_MODULE


#include "CtrlThread.h"


class CtrlModule : public RFModule
{
protected:
    CtrlThread *thr;

public:
    virtual bool configure(ResourceFinder &rf);

    virtual bool close();

    virtual double getPeriod();
    virtual bool   updateModule();
};

#endif
