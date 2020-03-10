#ifndef CTRL_THREAD
#define CTRL_THREAD

#include <cstdio>
#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>


#define CTRL_THREAD_PER     0.02    // [s]
#define PRINT_STATUS_PER    1.0     // [s]
#define MAX_TORSO_PITCH     30.0    // [deg]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class CtrlThread: public PeriodicThread,
                  public CartesianEvent
{
protected:

    PolyDriver         client;
    PolyDriver        clientGaze;
    IGazeControl     *igaze;
    IEncoders        *ienc;
    ICartesianControl *icart;
    BufferedPort<Bottle> inPort;

    Vector xd;
    Vector od;
    Vector fp;

    int startup_context_id;

    double t;
    double t0;
    double t1;

    // the event callback attached to the "motion-ongoing"
    virtual void cartesianEventCallback();

public:
    CtrlThread(const double period);

    virtual bool threadInit();

    virtual void afterStart(bool s);

    virtual void run();

    virtual void threadRelease();

    void generateTarget();

    void limitTorsoPitch();

    void printStatus();

    void initWorld();

};

#endif
