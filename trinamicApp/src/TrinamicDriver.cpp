/*

FILENAME... TrinamicDriver.cpp
USAGE...    Motor driver support for any Trinamic controller that uses the TMCL language.
	Note that this must be in ASCII mode and not binary mode.

Bryan Monk
March 14, 2023

Based on https://github.com/epics-motor/motorAcs/blob/master/acsApp/src/MCB4BDriver.cpp
	by Mark Rivers


*/

/*

TODO: add support for STAP? (store axis parameter)

CONTROLLER NOTES:
 
some parameters use internal units - refer to TMCL firmware manual p. 124 for conversion

Command format (bytes):
1. module address - default is 0x01 for serial (1)
2. command number (1)
3. type number (1)
4. motor or bank number (1)
5. value - msb first (4)
6. checksum (1)

Reply format:
1. (1) Reply address
2. (1) Module address
3. (1) Status (100 = no error)
4. (1) Command number
5. (4) Value (MSB first)
6. (1) checksum

// TODO: add multiple address suppport

Command List:

set vel: <address> 05 04 <motor #> <vel (4)> <checksum>
set accl: <address> 05 05 <motor #> <accel (4)> <checksum>
move abs: <address> 04 00 <motor #> <position (4)> <checksum> 
move rel: <address> 04 01 <motor #> <rel position (4)> <checksum>
move vel: <address> 01/02 00 (rotate right/left) <vel position (4)> <checksum> 
stop: <address> 03 00 <motor #> 00 00 00 00 <checksum>
set pos: <address> 05 01 <motor #> <position (4)> <checksum> 
get pos: <address> 06 01 <motor #> 00 00 00 00 <checksum> 
get moving status (position reached flag, 0 if moving): <address> 06 08 <motor #> 00 00 00 00 <checksum>
	maybe check speed param instead? (GAP 0x03)
get lim status: <address> 06 0A/0B (right/left) <motor #> 00 00 00 00 <checksum>
get drive pwr on (?): NOT SUPPORTED: always on - unless turn current to 0?
set pulse div: <address> 05 9A <motor #> <value (4)> <checksum>
set ramp div: <address> 05 99 <motor #> <value (4)> <checksum>

Units: 
position: microsteps
velocity: internal units (int)
accel: internal units (int)

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <TrinamicDriver.h>

#include <asynOctetSyncIO.h>

#include <asynPortDriver.h>
#include <shareLib.h>

#include <iocsh.h>
#include <epicsExport.h>
#include <epicsThread.h>

// function for converting double -> int
#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

// set checksum directly in function or return number?
void TrinamicController::calcTrinamicChecksum(char* command)
{
    char checksum=0;
    
    for(int i=0; i<8; i++) {
        checksum += command[i];	
    }
    command[8] = checksum;
}

// convert velocity double (usteps/s) to int used in trinamic controller (-2047...2047)
int TrinamicController::vel_steps_to_int (double velocity, unsigned int pulse_div)
{
    double v_double;
    int v_int;
    
    v_double = 0.004096 * (double)(1UL << pulse_div) * velocity;

    if (v_double > 2047) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
    	    "Commanded velocity of %f (%d after conversion) above max value, writing max velocity\n", 
            velocity, (int)v_double);
        v_double = 2047;
    }
    else if (v_double < -2047) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
    	    "Commanded velocity of %f (%d after conversion) below min value, writing min velocity\n", 
            velocity, (int)v_double);
        v_double = -2047;
    }
    
    v_int = NINT(v_double);
    
    return v_int;
}

// convert accel double (usteps/s^2) to int used in trinamic controller (1...2047)
unsigned int TrinamicController::accel_steps_to_int (double acceleration, unsigned int pulse_div, 
        unsigned int ramp_div)
{
    double a_double;
    unsigned int a_int;
    
    a_double = (3.90625e-15) * (double)(1UL << (ramp_div+pulse_div+29)) * acceleration;
    
    if (a_double > 2047) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
    	    "Commanded acceleration of %f (%d after conversion) above max value, writing max acceleration\n", 
            acceleration, (int)a_double);
        a_double = 2047;
    }
    else if (a_double < 1) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
    	    "Commanded acceleration of %f (%d after conversion) below min value, writing min acceleration\n", 
            acceleration, (int)a_double);
        a_double = 1;
    }
    
    a_int = NINT(a_double);
    
    return a_int;
}

/** Creates a new TrinamicController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] TrinamicPortName  The name of the drvAsynSerialPort that was created previously to connect to the Trinamic controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  * \param[in] pulse_div         The time between polls when no axis is moving 
  * \param[in] ramp_div          Value of pulse divisor to be sent with each move
  * \param[in] run_current       Value of moving current to be sent on init (0...255) 
  * \param[in] standby_current   Value of standby current to be sent on init (0...255)
  * \param[in] ustep_res         Value of microstep resolution (0...8) 
  */

TrinamicController::TrinamicController(const char* portName, const char* TrinamicPortName,
        int numAxes, double movingPollPeriod, double idlePollPeriod, 
        // unsigned int pulse_div, 
        // unsigned int ramp_div, unsigned int run_current, unsigned int standby_current, 
        // unsigned int ustep_res, 
        char module_addr, const char* model, int limMask)
        : asynMotorController(portName, numAxes, NUM_TRINAMIC_PARAMS, 
                0, // No additional callback interfaces beyond those in base class
                0,
                ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                1, // autoconnect
                0, 0)  // Default priority and stack size
{
    int axis;
    asynStatus status;
    TrinamicAxis* pAxis;
    static const char* functionName = "TrinamicController::TrinamicController";


    for (int i = 0; i < 6; i++) {
        char paramName[50];
    
        sprintf(paramName, "%s_%d", PulseDivString_BASE, i);
        createParam(paramName, asynParamInt32, &PulseDiv_[i]);
    
        sprintf(paramName, "%s_%d", RampDivString_BASE, i);
        createParam(paramName, asynParamInt32, &RampDiv_[i]);
    
        sprintf(paramName, "%s_%d", RunCurrentString_BASE, i);
        createParam(paramName, asynParamInt32, &RunCurrent_[i]);
    
        sprintf(paramName, "%s_%d", StandbyCurrentString_BASE, i);
        createParam(paramName, asynParamInt32, &StandbyCurrent_[i]);
    
        sprintf(paramName, "%s_%d", UStepResString_BASE, i);
        createParam(paramName, asynParamInt32, &UStepRes_[i]);
    
        sprintf(paramName, "%s_%d", PulseDivRBVString_BASE, i);
        createParam(paramName, asynParamInt32, &PulseDiv_RBV_[i]);
    
        sprintf(paramName, "%s_%d", RampDivRBVString_BASE, i);
        createParam(paramName, asynParamInt32, &RampDiv_RBV_[i]);
    
        sprintf(paramName, "%s_%d", RunCurrentRBVString_BASE, i);
        createParam(paramName, asynParamInt32, &RunCurrent_RBV_[i]);
    
        sprintf(paramName, "%s_%d", StandbyCurrentRBVString_BASE, i);
        createParam(paramName, asynParamInt32, &StandbyCurrent_RBV_[i]);
    
        sprintf(paramName, "%s_%d", UStepResRBVString_BASE, i);
        createParam(paramName, asynParamInt32, &UStepRes_RBV_[i]);
    }

    // Connect to Trinamic Controller
    status = pasynOctetSyncIO->connect(TrinamicPortName, 0, &pasynUserController_, NULL);
    if(status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
    	    "%s: cannot connect to Trinamic controller\n", functionName);
    }
    
    // set controller specific parameters:
    // this->pulse_div = pulse_div;
    // this->ramp_div = ramp_div;
    // this->run_current = run_current;
    // this->standby_current = standby_current;
    // this->ustep_res = ustep_res;
    this->module_addr = module_addr;
    this->limMask = limMask;

    this->numAxes = numAxes;

    if (strcmp(model, "6110") == 0) this->model = "6110";
    else if (strcmp(model, "6214") == 0) this->model = "6214";

    // TODO: set default values of asyn parameters
    
    for(axis=0; axis<numAxes; axis++)
    {
        pAxis = new TrinamicAxis(this, axis);	
    }
    
    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Creates a new TrinamicController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] TrinamicPortName       The name of the drvAsynIPPPort that was created previously to connect to the Trinamic controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */

extern "C" int TrinamicCreateController(const char* portName, const char* TrinamicPortName, 
        int numAxes, int movingPollPeriod, int idlePollPeriod, 
        // unsigned int pulse_div, 
        // unsigned int ramp_div, unsigned int run_current, unsigned int standby_current,
        // unsigned int ustep_res, 
        char module_addr, const char* model, int limMask)
{
    // // check paramters are within valid range:
    // if ((pulse_div > 13) || (ramp_div > 13) ||
    //     (run_current > 255) || (standby_current > 255) ||
    //     (ustep_res > 8)) 
    // {
    //     return(asynError);
    // }
    
    TrinamicController* pTrinamicController = new TrinamicController(portName, TrinamicPortName,
            numAxes, movingPollPeriod/1000., idlePollPeriod/1000., 
            // pulse_div, ramp_div,
            // run_current, standby_current, ustep_res, 
            module_addr, model, limMask);
    pTrinamicController = NULL;

    return(asynSuccess);
}

void TrinamicController::report(FILE *fp, int level)
{
    fprintf(fp, "Trinamic motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
        this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);
    
    // Call the base class method
    asynMotorController::report(fp, level);
}

/** Returns a pointer to an TrinamicAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
TrinamicAxis* TrinamicController::getAxis(asynUser *pasynUser)
{
    return static_cast<TrinamicAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an TrinamicAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
TrinamicAxis* TrinamicController::getAxis(int axisNo)
{
    return static_cast<TrinamicAxis*>(asynMotorController::getAxis(axisNo));
}

asynStatus TrinamicController::writeReadController()
{
    size_t nread;
    return writeReadController(outString_, inString_, TRINAMIC_CMD_SIZE,
        &nread, DEFAULT_CONTROLLER_TIMEOUT);
}

asynStatus TrinamicController::writeReadController(const char *output, char *input, 
                                    size_t maxChars, size_t *nread, double timeout)
{
    size_t nwrite;
    asynStatus status;
    int eomReason;
    
    status = pasynOctetSyncIO->writeRead(pasynUserController_, output,
                    TRINAMIC_CMD_SIZE, input, maxChars, timeout,
                    &nwrite, nread, &eomReason);
                          
    return status;
}

// most TMCL commands send a 32 bit integer
asynStatus TrinamicController::sendIntTMCL(char arg0, char arg1, char arg2, char arg3, int val) {

    asynStatus status;

    // set bytes of outString
    outString_[0] = arg0;
    outString_[1] = arg1;
    outString_[2] = arg2;
    outString_[3] = arg3;
                        
    outString_[4] = (char)(0xFF & (val >> 24));
    outString_[5] = (char)(0xFF & (val >> 16));
    outString_[6] = (char)(0xFF & (val >> 8 ));
    outString_[7] = (char)(0xFF & (val      ));		
    
    // set next byte of outString_ to checksum needed for command
    calcTrinamicChecksum(outString_);
    
    // send outString_ to controller
    status = writeReadController();

    return status;
}

// These are the TrinamicAxis methods

/** Creates a new TrinamicAxis object.
  * \param[in] pC Pointer to the TrinamicController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC_->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */

TrinamicAxis::TrinamicAxis(TrinamicController *pC, int axisNo)
        : asynMotorAxis(pC, axisNo), pC_(pC)
{  
    // inital commands to send:
    asynStatus status;
    
    // enable left limit switch
    // format: <address> 05 0D <motor #> <0 (enable) (4 bytes)> <checksum>
    status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0x0D, (char)axisNo_, 0);

    // enable right limit switch
    // format: <address> 05 0C <motor #> <0 (enable) (4 bytes)> <checksum>
    status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0x0C, (char)axisNo_, 0);

    // limMask format: last 12 bits represent
    // 1 for inverted, 0 for regular (normally closed)
    // TODO; confirm that it's setting the right limit polarity
    //
    int rightLimPolarity, leftLimPolarity;

    if (pC_->model == "6110") {
        // 6110 only has global limit switch polarity setting. This will run for each axis creation but doesn't need to
        char globalLimByte = ((pC_->limMask & 1) == 1) ? 3 : 0;
        status = pC_->sendIntTMCL(pC_->module_addr, 0x09, 79, 0x00, globalLimByte);
	// ************************************
	// reverse shaft
        // status = pC_->sendIntTMCL(pC_->module_addr, 0x09, 90, 0x00, 3);
	// ************************************
    }
    // 6214 has axis specific limit polarity
    else if (pC_->model == "6214") {
        rightLimPolarity = (pC_->limMask >> (2*axisNo)) & 1;
        leftLimPolarity = (pC_->limMask >> (2*axisNo + 1)) & 1;
        // set right limit switch polarity (05 18)
        status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0x18, (char)axisNo_, rightLimPolarity);
        // set left limit switch polarity (05 19)
        status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0x19, (char)axisNo_, leftLimPolarity);
    }

}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void TrinamicAxis::report(FILE *fp, int level)
{
    if (level > 0) {
        fprintf(fp, "  axis %d\n", axisNo_);
    }
    
    // Call the base class method
    asynMotorAxis::report(fp, level);
}

// helper function to get limits since used twice and need to read before moving
asynStatus TrinamicAxis::getLimits()
{
    asynStatus status;
    int leftLimit, rightLimit;
    
    // get limit polarity 
    // TODO: polarity calc might be wrong?
    int rightLimPolarity = (pC_->limMask >> (2*axisNo_)) & 1;
    int leftLimPolarity = (pC_->limMask >> (2*axisNo_ + 1)) & 1;

    // Read the limit status
    // get lim status: <address> 06 0A/0B (right/left) <motor #> 00 00 00 00 <checksum>

    // if limit mask for axis is 0 (noninverted), limit read of 1 = triggered 
    // if limit mask for axis is 1 (inverted), limit read of 0 = triggered 

    // get right limit
    status = pC_->sendIntTMCL(pC_->module_addr, 0x06, 0x0A, (char)axisNo_, 0);

    if (status) return status;
    // 1 means switch is activated
    rightLimit = (int)(pC_->inString_[7] & 0x000000FF);
    if (pC_->model == "6214") rightLimit = !rightLimit;
    setIntegerParam(pC_->motorStatusHighLimit_, rightLimit);
    
    // get left limit
    status = pC_->sendIntTMCL(pC_->module_addr, 0x06, 0x0B, (char)axisNo_, 0);

    if (status) return status;
    // 1 means switch is activated
    leftLimit = (int)(pC_->inString_[7] & 0x000000FF);
    if (pC_->model == "6214") leftLimit = !leftLimit;
    setIntegerParam(pC_->motorStatusLowLimit_, leftLimit);
    
    // // cancel move (stop) if limit hit
    // // default behavior is that if limit is lifted, move will continue
    // // if ((rightLimit || leftLimit) && !(pastLeftLimit || pastRightLimit)) comStatus = stop(0);
    // if ((rightLimit && !pastRightLimit) || (leftLimit && !pastLeftLimit)) comStatus = stop(0);
    // pastLeftLimit = leftLimit;
    // pastRightLimit = rightLimit;
    return status;
}

asynStatus TrinamicAxis::sendAccelAndVelocity(double acceleration, double velocity)
{
    asynStatus status;
    
    int accel_int;
    int vel_int;
    int pulse_div, ramp_div;

    if (pC_->model == "6110") {
        // convert velocity and accel from microsteps to controller units for 6110
	pC_->getIntegerParam(pC_->PulseDiv_[axisNo_], &pulse_div);
	pC_->getIntegerParam(pC_->RampDiv_[axisNo_], &ramp_div);
        vel_int = pC_->vel_steps_to_int(velocity, pulse_div);	
        accel_int = pC_->accel_steps_to_int(acceleration, pulse_div, ramp_div);	

        // // set pulse divisor
        // // format: <address> 05 9A <motor #> <value (4)> <checksum>
        // status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0x9A, (char)axisNo_, pC_->pulse_div);
        // 
        // // set ramp divisor
        // // format: <address> 05 99 <motor #> <value (4)> <checksum>
        // status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0x99, (char)axisNo_, pC_->ramp_div);
    }
    else if (pC_->model == "6214") {
        vel_int = NINT(velocity);
        accel_int = NINT(acceleration);
        
        // set deceleration = acceleration (0x11)
        status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0x11, (char)axisNo_, accel_int);
        // set start velocity = 0 (0x13)
        status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0x13, (char)axisNo_, 1);
        // set stop velocity = 0 (0x14)
        status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0x14, (char)axisNo_, 1);
        // set A1 = 0 (0x0F)
        status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0x0F, (char)axisNo_, 0);
        // set V1 = 0 (0x10)
        status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0x10, (char)axisNo_, 0);
        // set D1 = acceleration (0x12)
        status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0x12, (char)axisNo_, accel_int);
        
    }

    // set velocity: <address> 05 04 <motor #> <vel (4)> <checksum>
    status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0x04, (char)axisNo_, vel_int);
    
    // set acceleration: <address> 05 05 <motor #> <accel (4)> <checksum>
    status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0x05, (char)axisNo_, accel_int);
    
    return status;
}

asynStatus TrinamicAxis::move(double position, int relative, double minVelocity, double maxVelocity,
        double acceleration)
{
    asynStatus status;
    
    int leftLimit, rightLimit;
    int targetDir;
    double curPosition;
    
    // abort move if out of range
    if ((position < -2147483648) || (position > 2147483647)) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
            "Position of %f out of range\n", position);
        return asynError; 
    }
    
    status = getLimits();
    pC_->getIntegerParam((int)axisNo_, pC_->motorStatusLowLimit_, &leftLimit);
    pC_->getIntegerParam((int)axisNo_, pC_->motorStatusHighLimit_, &rightLimit);
    
    pC_->getDoubleParam((int)axisNo_, pC_->motorPosition_, &curPosition);
    
    // keep track of direction sent
    if (relative) { 
        targetDir = (position >= 0)? 1 : -1;
    }
    else {
        targetDir = (position >= curPosition)? 1 : -1;
    }
    
    // don't send move if it would be in direction that limit is activated
    if ((leftLimit && (targetDir == -1)) || (rightLimit && (targetDir == 1)))
        return status;
    
    // set acceleration and velocity:
    status = sendAccelAndVelocity(acceleration, maxVelocity);
    
    // convert position to controller units 
    int pos_int = NINT(position);
    
    // send move command:
    if (relative) {
    	// move rel: <address> 04 01 <motor #> <rel position (4)> <checksum>
        status = pC_->sendIntTMCL(pC_->module_addr, 0x04, 0x01, (char)axisNo_, pos_int);
    }
    else
    {
        // move abs: <address> 04 00 <motor #> <position (4)> <checksum> 
        status = pC_->sendIntTMCL(pC_->module_addr, 0x04, 0x00, (char)axisNo_, pos_int);
    }
    
    return status;
    
}

asynStatus TrinamicAxis::home(double minVelocity, double maxVelocity, 
		double acceleration, int forwards)
{
    asynStatus status;
    // home (aka start reference search RFS): <address> 0D 00 <motor #> <0 (4)> <checksum>
    this->homingInProg = 1;
    
    // TODO: controller acts weird if send accel and vel before homing
    // status = sendAccelAndVelocity(acceleration, maxVelocity);
    
    // set reference search mode: <address> 05 05 <motor #> <accel (4)> <checksum>
    
    if (forwards) {
        // set search mode to find right switch 
        status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0xC1, (char)axisNo_, 0x00000041);
    }
    else {
        // set search mode to find left switch 
        status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0xC1, (char)axisNo_, 1);
    }
    
    
    status = pC_->sendIntTMCL(pC_->module_addr, 0x0D, 0x00, (char)axisNo_, 0);

    status = pC_->writeReadController();
    return status;
}

asynStatus TrinamicAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
    asynStatus status;
    double absVelocity = (maxVelocity >= 0.) ? maxVelocity : -1.0*maxVelocity;
    int pulse_div;
    
    // send accel and velocity
    status = sendAccelAndVelocity(acceleration, absVelocity);
    
    int vel_int;
    if (pC_->model == "6110") {
	pC_->getIntegerParam(pC_->PulseDiv_[axisNo_], &pulse_div);
        vel_int = pC_->vel_steps_to_int(absVelocity, pulse_div);	
    }
    if (pC_->model == "6214") {
        vel_int = NINT(absVelocity);
    }

    // if velocity negative, send rotate left (ROL) command, else send rotate right (ROR)
    // format: <address> 01/02 (ROL/ROR) 00 <motor #> <velocity (4)> <checksum>
    if (maxVelocity >= 0) {
        
        pC_->lock();
        status = pC_->sendIntTMCL(pC_->module_addr, 0x01, 0x00, (char)axisNo_, vel_int);
        epicsThreadSleep(0.1);
        pC_->unlock();
    }
    else
    {
        pC_->lock();
        status = pC_->sendIntTMCL(pC_->module_addr, 0x02, 0x00, (char)axisNo_, vel_int);
        epicsThreadSleep(0.1);
        pC_->unlock();
    }
    
    // pC_->lock();
    // status = pC_->writeReadController();
    // epicsThreadSleep(0.1);
    // pC_->unlock();
    
    return status;
}

asynStatus TrinamicAxis::stop(double acceleration)
{
    
    asynStatus status;
    
    // unsigned int accel_int;
    // accel_int = pC_->accel_steps_to_int(acceleration, pC_->pulse_div, pC_->ramp_div);	
    // 
    // // set accl: <address> 05 05 <motor #> <accel (4)> <checksum>
    // pC_->outString_[0] = pC_->module_addr;
    // pC_->outString_[1] = 0x05;
    // pC_->outString_[2] = 0x05;
    // pC_->outString_[3] = (char)axisNo_;
    // 
    // // set 4 bytes of accel 
    // pC_->outString_[4] = (char)((accel_int & 0xFF000000) >> 24);
    // pC_->outString_[5] = (char)((accel_int & 0x00FF0000) >> 16);
    // pC_->outString_[6] = (char)((accel_int & 0x0000FF00) >> 8);
    // pC_->outString_[7] = (char)(accel_int & 0x000000FF);		
    // 
    // pC_->calcTrinamicChecksum(pC_->outString_);
    // 
    // status = pC_->writeReadController();
    // if (status) return status;
    
    // stop: <address> 03 00 <motor #> 00 00 00 00 <checksum>
    status = pC_->sendIntTMCL(pC_->module_addr, 0x03, 0x00, (char)axisNo_, 0);
    // printf("sent stop, status = %d\n", status);
    
    return status;
}

asynStatus TrinamicAxis::setPosition(double position)
{
    asynStatus status;
    
    // send stop command, motor moves when setting position otherwise
    status = pC_->sendIntTMCL(pC_->module_addr, 0x03, 0x00, (char)axisNo_, 0);
    	
    if (status) {
        return asynError;
    }
    
    int pos_int = NINT(position);
    
    // set pos: <address> 05 01 <motor #> <position (4)> <checksum> 
    status = pC_->sendIntTMCL(pC_->module_addr, 0x05, 0x01, (char)axisNo_, pos_int);
    return status;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status,
  * and the drive power-on status.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). 
*/

asynStatus TrinamicAxis::poll(bool *moving)
{
    int done;
    // int driveOn;
    double position;
    int prevLeftLimit, prevRightLimit, curLeftLimit, curRightLimit; 
    int curDir;
    double accel;
    int pulse_div, ramp_div, run_cur, stand_cur, ustep_res; // extra asyn params
    
    asynStatus status;

    // readback of extra asyn params:
    
    if (pC_->model == "6110") {
    	// pulse div
    	status = pC_->sendIntTMCL(pC_->module_addr, 0x06, 0x9A, (char)axisNo_, 0);
    	pulse_div = (int)(
    	    ((pC_->inString_[4] << 24) & 0xFF000000) | 
    	    ((pC_->inString_[5] << 16) & 0x00FF0000) | 
    	    ((pC_->inString_[6] << 8 ) & 0x0000FF00) | 
    	    ((pC_->inString_[7] << 0 ) & 0x000000FF) );

    	pC_->setIntegerParam(pC_->PulseDiv_RBV_[axisNo_], pulse_div);

    	// ramp div
    	status = pC_->sendIntTMCL(pC_->module_addr, 0x06, 0x99, (char)axisNo_, 0);
    	ramp_div = (int)(
    	    ((pC_->inString_[4] << 24) & 0xFF000000) | 
    	    ((pC_->inString_[5] << 16) & 0x00FF0000) | 
    	    ((pC_->inString_[6] << 8 ) & 0x0000FF00) | 
    	    ((pC_->inString_[7] << 0 ) & 0x000000FF) );

    	pC_->setIntegerParam(pC_->RampDiv_RBV_[axisNo_], ramp_div);
    }

    // run current
    status = pC_->sendIntTMCL(pC_->module_addr, 0x06, 0x06, (char)axisNo_, 0);
    run_cur = (int)(
        ((pC_->inString_[4] << 24) & 0xFF000000) | 
        ((pC_->inString_[5] << 16) & 0x00FF0000) | 
        ((pC_->inString_[6] << 8 ) & 0x0000FF00) | 
        ((pC_->inString_[7] << 0 ) & 0x000000FF) );

    pC_->setIntegerParam(pC_->RunCurrent_RBV_[axisNo_], run_cur);
     
    // standby current
    status = pC_->sendIntTMCL(pC_->module_addr, 0x06, 0x07, (char)axisNo_, 0);
    stand_cur = (int)(
        ((pC_->inString_[4] << 24) & 0xFF000000) | 
        ((pC_->inString_[5] << 16) & 0x00FF0000) | 
        ((pC_->inString_[6] << 8 ) & 0x0000FF00) | 
        ((pC_->inString_[7] << 0 ) & 0x000000FF) );

    pC_->setIntegerParam(pC_->StandbyCurrent_RBV_[axisNo_], stand_cur);

    // ustep res
    status = pC_->sendIntTMCL(pC_->module_addr, 0x06, 0x8C, (char)axisNo_, 0);
    ustep_res = (int)(
        ((pC_->inString_[4] << 24) & 0xFF000000) | 
        ((pC_->inString_[5] << 16) & 0x00FF0000) | 
        ((pC_->inString_[6] << 8 ) & 0x0000FF00) | 
        ((pC_->inString_[7] << 0 ) & 0x000000FF) );

    pC_->setIntegerParam(pC_->UStepRes_RBV_[axisNo_], ustep_res);

    
    // Read the current motor position
    // get pos: <address> 06 01 <motor #> 00 00 00 00 <checksum> 
    status = pC_->sendIntTMCL(pC_->module_addr, 0x06, 0x01, (char)axisNo_, 0);

    if (status) goto skip;
    // value of a read is in bytes 4-7	
    // response placed in pC_->inString_
    // convert 8 bytes to double and set parameter
    position = (double) (int)(
        ((pC_->inString_[4] << 24) & 0xFF000000) | 
        ((pC_->inString_[5] << 16) & 0x00FF0000) | 
        ((pC_->inString_[6] << 8 ) & 0x0000FF00) | 
        ((pC_->inString_[7] << 0 ) & 0x000000FF) );
    
    setDoubleParam(pC_->motorPosition_, position);
    
    // check if velocity != 0
    status = pC_->sendIntTMCL(pC_->module_addr, 0x06, 0x03, (char)axisNo_, 0);

    if (status) goto skip;
    
    // check if any velocity byte is not 0
    done = !(pC_->inString_[4] || pC_->inString_[5] || pC_->inString_[6] || pC_->inString_[7]);
    
    // get current direction from MSB byte of velocity
    if (done) {
        curDir = 0;
    }
    else {
        curDir = ((int)pC_->inString_[4] < 0) ? -1 : 1;
    }
    
    setIntegerParam(pC_->motorStatusDone_, done);
    *moving = done ? false : true;	
    
    // store previous values of limits
    pC_->getIntegerParam((int)axisNo_, pC_->motorStatusLowLimit_, &prevLeftLimit);
    pC_->getIntegerParam((int)axisNo_, pC_->motorStatusHighLimit_, &prevRightLimit);
    
    // Read the limit status
    status = getLimits();
    
    // store current limit values
    pC_->getIntegerParam((int)axisNo_, pC_->motorStatusLowLimit_, &curLeftLimit);
    pC_->getIntegerParam((int)axisNo_, pC_->motorStatusHighLimit_, &curRightLimit);
    
    // cancel move (stop) if limit hit
    // default behavior is that if limit is lifted, move will continue
    pC_->getDoubleParam((int)axisNo_, pC_->motorAccel_, &accel);
    
    if ((curRightLimit && !prevRightLimit && ((curDir == 1) || (done == 1))) ||
        (curLeftLimit && !prevLeftLimit && ((curDir == -1) || (done == 1)))) {
        // don't cancel motion if homing
        if (this->homingInProg) {
            this->homingInProg = 0;
            goto skip;
        }
        status = stop(accel);
    }
    
    skip:
    setIntegerParam(pC_->motorStatusProblem_, status ? 1:0);
    callParamCallbacks();
    pC_->callParamCallbacks(); // controller callbacks for asyn params
    return status ? asynError : asynSuccess;
}

// asyn functions for extra params:
asynStatus TrinamicController::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	static const char *functionName = "writeInt32";

    // asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
	// 		"%s:%s, port %s, function = %d\n",
	// 		driverName, functionName, this->portName, function);

	// Set the new value; it can be reverted later if commands fail
	setIntegerParam(function, value);

    if      (function == PulseDiv_[0])         status = setPulseDiv(value, 0);
    else if (function == PulseDiv_[1])         status = setPulseDiv(value, 1);
    else if (function == PulseDiv_[2])         status = setPulseDiv(value, 2);
    else if (function == PulseDiv_[3])         status = setPulseDiv(value, 3);
    else if (function == PulseDiv_[4])         status = setPulseDiv(value, 4);
    else if (function == PulseDiv_[5])         status = setPulseDiv(value, 5);
    else if (function == RampDiv_[0])          status = setRampDiv(value, 0);
    else if (function == RampDiv_[1])          status = setRampDiv(value, 1);
    else if (function == RampDiv_[2])          status = setRampDiv(value, 2);
    else if (function == RampDiv_[3])          status = setRampDiv(value, 3);
    else if (function == RampDiv_[4])          status = setRampDiv(value, 4);
    else if (function == RampDiv_[5])          status = setRampDiv(value, 5);
    else if (function == RunCurrent_[0])       status = setRunCurrent(value, 0);
    else if (function == RunCurrent_[1])       status = setRunCurrent(value, 1);
    else if (function == RunCurrent_[2])       status = setRunCurrent(value, 2);
    else if (function == RunCurrent_[3])       status = setRunCurrent(value, 3);
    else if (function == RunCurrent_[4])       status = setRunCurrent(value, 4);
    else if (function == RunCurrent_[5])       status = setRunCurrent(value, 5);
    else if (function == StandbyCurrent_[0])   status = setStandbyCurrent(value, 0);
    else if (function == StandbyCurrent_[1])   status = setStandbyCurrent(value, 1);
    else if (function == StandbyCurrent_[2])   status = setStandbyCurrent(value, 2);
    else if (function == StandbyCurrent_[3])   status = setStandbyCurrent(value, 3);
    else if (function == StandbyCurrent_[4])   status = setStandbyCurrent(value, 4);
    else if (function == StandbyCurrent_[5])   status = setStandbyCurrent(value, 5);
    else if (function == UStepRes_[0])         status = setUStepRes(value, 0);
    else if (function == UStepRes_[1])         status = setUStepRes(value, 1);
    else if (function == UStepRes_[2])         status = setUStepRes(value, 2);
    else if (function == UStepRes_[3])         status = setUStepRes(value, 3);
    else if (function == UStepRes_[4])         status = setUStepRes(value, 4);
    else if (function == UStepRes_[5])         status = setUStepRes(value, 5);
    else asynMotorController::writeInt32(pasynUser, value);

	callParamCallbacks();

	// if (status == 0) {
	// 	asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
    //          "%s:%s, port %s, wrote %d\n",
    //          driverName, functionName, this->portName, value);
	// } else {
	// 	asynPrint(pasynUser, ASYN_TRACE_ERROR,
    //          "%s:%s, port %s, ERROR writing %d, status=%d\n",
    //          driverName, functionName, this->portName, value, status);
	// }

	return (status==0) ? asynSuccess : asynError;
}
    // 
    // 

asynStatus TrinamicController::setPulseDiv(epicsInt32 value, int axis) {
    static const char *functionName = "setPulseDiv";

    asynStatus comStatus = asynSuccess;

	// set pulse divisor
	// format: <address> 05 9A <motor #> <value (4)> <checksum>
	
	if (model == "6110") {
		comStatus = sendIntTMCL(module_addr, 0x05, 0x9A, (char)axis, (char)value);
	}

    // asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,
	// 	"%s:%s, port %s, value = %d\n",
	// 	driverName, functionName, this->portName, value);
	
    return comStatus;
}
asynStatus TrinamicController::setRampDiv(epicsInt32 value, int axis) {
    static const char *functionName = "setRampDiv";

    asynStatus comStatus = asynSuccess;

	// set ramp divisor
	// format: <address> 05 99 <motor #> <value (4)> <checksum>
	if (model == "6110") {
		comStatus = sendIntTMCL(module_addr, 0x05, 0x99, (char)axis, (char)value);
	}

    // asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,
	// 	"%s:%s, port %s, value = %d\n",
	// 	driverName, functionName, this->portName, value);

    return comStatus;
}
asynStatus TrinamicController::setRunCurrent(epicsInt32 value, int axis) {
    static const char *functionName = "setRunCurrent";

    asynStatus comStatus;

    // set run current (SAP) for all axes:
    // format: <address> 05 06 <motor #> <current (0-255)(4 bytes)> <checksum>
	comStatus = sendIntTMCL(module_addr, 0x05, 0x06, axis, (int)value);

    // asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,
	// 	"%s:%s, port %s, value = %d\n",
	// 	driverName, functionName, this->portName, value);

    return comStatus;
}
asynStatus TrinamicController::setStandbyCurrent(epicsInt32 value, int axis) {
    static const char *functionName = "setStandbyCurrent";

    asynStatus comStatus;
        
    // set standby current (SAP) for all axes:
    // format: <address> 05 07 <motor #> <current (0-255)(4 bytes)> <checksum>
        comStatus = sendIntTMCL(module_addr, 0x05, 0x07, axis, (int)value);

    // asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,
	// 	"%s:%s, port %s, value = %d\n",
	// 	driverName, functionName, this->portName, value);

    return comStatus;
}
asynStatus TrinamicController::setUStepRes(epicsInt32 value, int axis) {
    static const char *functionName = "setUStepRes";

    asynStatus comStatus;

    // set microstep resolution (SAP) for all axes:
    // format: <address> 05 8C <motor #> <resolution (0-8)(4 bytes)> <checksum>
    // # of microsteps will be 2^ustep_res steps per full step
        comStatus = sendIntTMCL(module_addr, 0x05, 0x8C, axis, (int)value);

    // asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,
	// 	"%s:%s, port %s, value = %d\n",
	// 	driverName, functionName, this->portName, value);

    return comStatus;
}



/** Code for iocsh registration */
static const iocshArg TrinamicCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg TrinamicCreateControllerArg1 = {"Trinamic port name", iocshArgString};
static const iocshArg TrinamicCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg TrinamicCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg TrinamicCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg TrinamicCreateControllerArg5 = {"module address", iocshArgInt};
static const iocshArg TrinamicCreateControllerArg6 = {"model", iocshArgString};
static const iocshArg TrinamicCreateControllerArg7 = {"lim polarity mask", iocshArgInt};
static const iocshArg * const TrinamicCreateControllerArgs[] = {&TrinamicCreateControllerArg0,
                                                             &TrinamicCreateControllerArg1,
                                                             &TrinamicCreateControllerArg2,
                                                             &TrinamicCreateControllerArg3,
                                                             &TrinamicCreateControllerArg4,
                                                             &TrinamicCreateControllerArg5,
                                                             &TrinamicCreateControllerArg6,
                                                             &TrinamicCreateControllerArg7};
static const iocshFuncDef TrinamicCreateControllerDef = {"TrinamicCreateController", 8, 
    TrinamicCreateControllerArgs};

static void TrinamicCreateControllerCallFunc(const iocshArgBuf *args)
{
    TrinamicCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, 
        args[4].ival, args[5].ival, args[6].sval, args[7].ival);
        // args[6].ival, args[7].ival, args[8].ival, 
        // args[9].ival, args[10].ival);
}

static void TrinamicRegister(void)
{
    iocshRegister(&TrinamicCreateControllerDef, TrinamicCreateControllerCallFunc);
}

extern "C" {
    epicsExportRegistrar(TrinamicRegister);
}
