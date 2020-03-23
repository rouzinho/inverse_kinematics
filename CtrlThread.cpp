#include "CtrlThread.h"


// the event callback attached to the "motion-ongoing"
void CtrlThread::cartesianEventCallback()
{
    fprintf(stdout,"20%% of trajectory attained\n");
}

CtrlThread::CtrlThread(const double period): PeriodicThread(period)
{
    // we wanna raise an event each time the arm is at 20%
    // of the trajectory (or 80% far from the target)
    cartesianEventParameters.type="motion-ongoing";
    cartesianEventParameters.motionOngoingCheckPoint=0.2;
}

bool CtrlThread::threadInit()
{
        // open a client interface to connect to the cartesian server of the simulator
        // we suppose that:
        //
        // 1 - the iCub simulator is running
        //     (launch: iCub_SIM)
        //
        // 2 - the cartesian server is running
        //     (launch: yarprobotinterface --context simCartesianControl)
        //
        // 3 - the cartesian solver for the left arm is running too
        //     (launch: iKinCartesianSolver --context simCartesianControl --part right_arm)
        // 4 - BRING A NICE box at a table world mk box 0.8 0.5 0.5 0 0.3 0.4 1 1 1 FALSE
    Property option("(device cartesiancontrollerclient)");
    option.put("remote","/icubSim/cartesianController/right_arm");
    option.put("local","/cartesian_client/right_arm");

    Property optGaze("(device gazecontrollerclient)");
    optGaze.put("remote","/iKinGazeCtrl");
    optGaze.put("local","/gaze_client");

    if (!clientGaze.open(optGaze))
        return false;


    bool ok = inPort.open("/cartesian/in");

    if (!client.open(option))
        return false;

    // open the view
    clientGaze.view(igaze);

    // latch the controller context in order to preserve
    // it after closing the module
    // the context contains the tracking mode, the neck limits and so on.
    igaze->storeContext(&startup_context_id);
     // set trajectory time:
    igaze->setNeckTrajTime(0.4);
    igaze->setEyesTrajTime(0.3);

        // put the gaze in tracking mode, so that
        // when the torso moves, the gaze controller
        // will compensate for it
    igaze->setTrackingMode(true);
    //igaze->setStabilizationMode(true);

    //initialize the simulator : set up the objects and resting the left arm.
    initWorld();

    // open the view
    client.view(icart);

    // latch the controller context in order to preserve
    // it after closing the module
    // the context contains the dofs status, the tracking mode,
    // the resting positions, the limits and so on.
    icart->storeContext(&startup_context_id);

    // set trajectory time
    icart->setTrajTime(1.8);

    // get the torso dofs
    Vector newDof, curDof;
    icart->getDOF(curDof);
    newDof=curDof;

    // enable the torso yaw and pitch
    // disable the torso roll
    newDof[0]=1;
    newDof[1]=0;
    newDof[2]=1;

    // send the request for dofs reconfiguration
    icart->setDOF(newDof,curDof);

    // impose some restriction on the torso pitch
    limitTorsoPitch();

    // print out some info about the controller
    Bottle info;
    icart->getInfo(info);
    fprintf(stdout,"info = %s\n",info.toString().c_str());

    // register the event, attaching the callback
    icart->registerEvent(*this);

    xd.resize(3);
    od.resize(4);
    fp.resize(3);
    //gaze fixation point
    fp[0] = -0.50;                                    // x-component [m]
    fp[1] = 0.00;                                    // y-component [m]
    fp[2] = -0.10;                                    // z-component [m]

    igaze->lookAtFixationPoint(fp);

    return true;
}

void CtrlThread::afterStart(bool s)
{
    if (s)
        fprintf(stdout,"Thread started successfully\n");
    else
        fprintf(stdout,"Thread did not start\n");

    t=t0=t1=Time::now();
}

void CtrlThread::run()
{
    t=Time::now();

    generateTarget();

    // go to the target :)
    // (in streaming)
    //igaze->lookAtFixationPoint(fp);
    icart->goToPose(xd,od);

    // some verbosity
    //printStatus();
}

void CtrlThread::threadRelease()
{
    // we require an immediate stop
    // before closing the client for safety reason
    icart->stopControl();

    // it's a good rule to restore the controller
    // context as it was before opening the module
    icart->restoreContext(startup_context_id);

    client.close();
}

void CtrlThread::generateTarget()
{
    // translational target part: a circular trajectory
    // in the yz plane centered in [-0.3,-0.1,0.1] with radius=0.1 m
    // and frequency 0.1 Hz
    Bottle *in = inPort.read();
    if (in==NULL) {
       fprintf(stderr, "Failed to read message\n");
    }

    string s = in->get(0).toString();
    xd[0] = stof(s);
    s = in->get(1).toString();
    xd[1] = stof(s);
    s = in->get(2).toString();
    xd[2] = stof(s);
    //xd[2] = 0.05;

    fp[0] = -0.50;                                    // x-component [m]
    fp[1] = 0.00;                                    // y-component [m]
    fp[2] = 0.35;                                    // z-component [m]


    /*xd[0]=-0.3;
    xd[1]=0.2;
    xd[2]=0.1;

    string s = in->get(0).toString();
    od[0] = stof(s);
    s = in->get(1).toString();
    od[1] = stof(s);
    s = in->get(2).toString();
    od[2] = stof(s);
    s = in->get(3).toString();
    od[3] = stof(s);*/


    /*xd[0]=-0.3;
    xd[1]=-0.1+0.1*cos(2.0*M_PI*0.1*(t-t0));
    xd[2]=+0.1+0.1*sin(2.0*M_PI*0.1*(t-t0));*/

    // we keep the orientation of the left arm constant:
    // we want the middle finger to point forward (end-effector x-axis)
    // with the palm turned down (end-effector y-axis points leftward);
    // to achieve that it is enough to rotate the root frame of pi around z-axis
    //left arm
    //od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=M_PI;

    //right arm
    od[0]=0.0; od[1]=1.0; od[2]=0.0; od[3]=M_PI;

}

void CtrlThread::limitTorsoPitch()
{
    int axis=0; // pitch joint
    double min, max;

    // sometimes it may be helpful to reduce
    // the range of variability of the joints;
    // for example here we don't want the torso
    // to lean out more than 30 degrees forward

    // we keep the lower limit
    icart->getLimits(axis,&min,&max);
    icart->setLimits(axis,min,MAX_TORSO_PITCH);
}

void CtrlThread::printStatus()
{
    if (t-t1>=PRINT_STATUS_PER)
    {
        Vector x,o,xdhat,odhat,qdhat;

        // we get the current arm pose in the
        // operational space
        icart->getPose(x,o);

        // we get the final destination of the arm
        // as found by the solver: it differs a bit
        // from the desired pose according to the tolerances
        icart->getDesired(xdhat,odhat,qdhat);

        double e_x=norm(xdhat-x);
        double e_o=norm(odhat-o);

        fprintf(stdout,"+++++++++\n");
        fprintf(stdout,"xd          [m] = %s\n",xd.toString().c_str());
        fprintf(stdout,"xdhat       [m] = %s\n",xdhat.toString().c_str());
        fprintf(stdout,"x           [m] = %s\n",x.toString().c_str());
        fprintf(stdout,"od        [rad] = %s\n",od.toString().c_str());
        fprintf(stdout,"odhat     [rad] = %s\n",odhat.toString().c_str());
        fprintf(stdout,"o         [rad] = %s\n",o.toString().c_str());
        fprintf(stdout,"norm(e_x)   [m] = %g\n",e_x);
        fprintf(stdout,"norm(e_o) [rad] = %g\n",e_o);
        fprintf(stdout,"---------\n\n");

        t1=t;
    }
}

//init the world and robot for the sake of our experiment

void CtrlThread::initWorld()
{
   Network yarp_arm;
   Network yarp_world;
   const char *client_name = "/setup_icub";
   const char *server_name = "/icubSim/left_arm/rpc:i";
   const char *client_world = "/setup_world";
   const char *server_world = "/icubSim/world";

   RpcClient port;
   RpcClient port_world;
   Bottle cmd;
   Bottle response;
   port.open(client_name);
   port_world.open(client_world);
   yarp_arm.connect(client_name,server_name);
   yarp_world.connect(client_world,server_world);
   cmd.addString("set");
   cmd.addString("pos");
   cmd.addInt32(3);
   cmd.addInt32(20);
   port.write(cmd,response);
   cmd.clear();
   response.clear();
   cmd.addString("set");
   cmd.addString("pos");
   cmd.addInt32(0);
   cmd.addInt32(-10);
   port.write(cmd,response);
   cmd.clear();
   response.clear();
   cmd.addString("set");
   cmd.addString("pos");
   cmd.addInt32(1);
   cmd.addInt32(20);
   port.write(cmd,response);
   cmd.clear();
   response.clear();
   cmd.addString("world");
   cmd.addString("del");
   cmd.addString("all");
   port.write(cmd,response);
   cmd.clear();
   response.clear();

   //set up a table world mk box 0.8 0.5 0.5 0 0.3 0.4 1 1 1 FALSE
   cmd.addString("world");
   cmd.addString("mk");
   cmd.addString("box");
   cmd.addDouble(0.8);
   cmd.addDouble(0.5);
   cmd.addDouble(0.5);
   cmd.addDouble(0);
   cmd.addDouble(0.3);
   cmd.addDouble(0.4);
   cmd.addInt(1);
   cmd.addInt(1);
   cmd.addInt(1);
   cmd.addString("FALSE");
   port_world.write(cmd,response);




}
