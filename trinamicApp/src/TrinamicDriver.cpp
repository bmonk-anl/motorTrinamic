/*

FILENAME... TrinamicDriver.cpp
USAGE...    Motor driver support for any Trinamic controller that uses the TMCL language.
	Note that this must be in ASCII mode and not binary mode.

Bryan Monk
March 14, 2023

Based on https://github.com/epics-motor/motorAcs/blob/master/acsApp/src/TrinamicDriver.cpp
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
	int i;

	for(i=0; i<8; i++) {
		checksum += command[i];	
	}

	command[8] = checksum;
	//return checksum; 
}

// convert velocity double (usteps/s) to int used in trinamic controller (1...2047)
int TrinamicController::vel_steps_to_int (double velocity, unsigned int pulse_div)
{
	double v_double;
	int v_int;

	v_double = 0.004096 * (double)(1UL << pulse_div) * velocity;
	v_int = NINT(v_double);

	if (v_int > 2047){
        v_int = 2047;
    }
    else if (v_int < -2047){
        v_int = -2047;
    }

	return v_int;
}

// convert accel double (usteps/s^2) to int used in trinamic controller (1...2047)
unsigned int TrinamicController::accel_steps_to_int (double acceleration, unsigned int pulse_div, 
		unsigned int ramp_div)
{
	double a_double;
	unsigned int a_int;

	a_double = (3.90625e-15) * (double)(1UL << (ramp_div+pulse_div+29)) * acceleration;
	a_int = NINT(a_double);

	if (a_int > 2047) a_int = 2047;

	return a_int;
}

/** Creates a new TrinamicController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] TrinamicPortName     The name of the drvAsynSerialPort that was created previously to connect to the Trinamic controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */

TrinamicController::TrinamicController(const char* portName, const char* TrinamicPortName,
		int numAxes, double movingPollPeriod, double idlePollPeriod, unsigned int pulse_div, 
        unsigned int ramp_div, unsigned int run_current, unsigned int standby_current, 
        unsigned int ustep_res)
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

	// Connect to Trinamic Controller
	status = pasynOctetSyncIO->connect(TrinamicPortName, 0, &pasynUserController_, NULL);
	if(status) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
				"%s: cannot connect to Trinamic controller\n", functionName);
	}

    // set controller specific parameters:
    this->pulse_div = pulse_div;
    this->ramp_div = ramp_div;
    this->run_current = run_current;
    this->standby_current = standby_current;
    this->ustep_res = ustep_res;

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
		int numAxes, int movingPollPeriod, int idlePollPeriod, unsigned int pulse_div, 
        unsigned int ramp_div, unsigned int run_current, unsigned int standby_current,
        unsigned int ustep_res)
{
	TrinamicController* pTrinamicController = new TrinamicController(portName, TrinamicPortName,
			numAxes, movingPollPeriod/1000., idlePollPeriod/1000., pulse_div, ramp_div,
            run_current, standby_current, ustep_res);
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
	// const char *functionName="writeReadController";
	
	// status = pasynOctetSyncIO->writeRead(pasynUserController_, output,
	//                                     strlen(output), input, maxChars, timeout,
	//                                     &nwrite, nread, &eomReason);
	
	status = pasynOctetSyncIO->writeRead(pasynUserController_, output,
	                                     TRINAMIC_CMD_SIZE, input, maxChars, timeout,
	                                     &nwrite, nread, &eomReason);
	                      
	return status;
}


// These are the TrinamicAxis methods

/** Creates a new TrinamicAxis object.
  * \param[in] pC Pointer to the TrinamicController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
TrinamicAxis::TrinamicAxis(TrinamicController *pC, int axisNo)
	: asynMotorAxis(pC, axisNo), pC_(pC)
{  
    // inital commands to send:
    asynStatus status;
    
    // set run current (SAP):
    // format: <address> 05 06 <motor #> <current (0-255)(4 bytes)> <checksum>
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x05;
	pC_->outString_[2] = 0x06;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value (only need to set last byte, max is 255)
	pC_->outString_[4] = 0; 
	pC_->outString_[5] = 0; 
	pC_->outString_[6] = 0; 
	pC_->outString_[7] = (char)(pC_->run_current & 0x000000FF);		

	pC_->calcTrinamicChecksum(pC_->outString_);

	status = pC_->writeReadController();

    // set standby current (SAP):
    // format: <address> 05 07 <motor #> <current (0-255)(4 bytes)> <checksum>
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x05;
	pC_->outString_[2] = 0x07;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value (only need to set last byte, max is 255)
	pC_->outString_[4] = 0; 
	pC_->outString_[5] = 0; 
	pC_->outString_[6] = 0; 
	pC_->outString_[7] = (char)(pC_->standby_current & 0x000000FF);		

	pC_->calcTrinamicChecksum(pC_->outString_);

	status = pC_->writeReadController();
    
    // set microstep resolution (SAP)
    // format: <address> 05 8C <motor #> <resolution (0-8)(4 bytes)> <checksum>
    // # of microsteps will be 2^ustep_res steps per full step
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x05;
	pC_->outString_[2] = 0x8C;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value (only need to set last byte, max is 255)
	pC_->outString_[4] = 0; 
	pC_->outString_[5] = 0; 
	pC_->outString_[6] = 0; 
	pC_->outString_[7] = (char)(pC_->ustep_res & 0x000000FF);		

	pC_->calcTrinamicChecksum(pC_->outString_);

	status = pC_->writeReadController();
    
    // enable left limit switch
    // format: <address> 05 0D <motor #> <0 (enable) (4 bytes)> <checksum>
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x05;
	pC_->outString_[2] = 0x0D;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value (only need to set last byte, max is 255)
	pC_->outString_[4] = 0; 
	pC_->outString_[5] = 0; 
	pC_->outString_[6] = 0; 
	pC_->outString_[7] = 0; 

	pC_->calcTrinamicChecksum(pC_->outString_);

	status = pC_->writeReadController();
    
    // enable right limit switch
    // format: <address> 05 0C <motor #> <0 (enable) (4 bytes)> <checksum>
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x05;
	pC_->outString_[2] = 0x0C;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value (only need to set last byte, max is 255)
	pC_->outString_[4] = 0; 
	pC_->outString_[5] = 0; 
	pC_->outString_[6] = 0; 
	pC_->outString_[7] = 0; 

	pC_->calcTrinamicChecksum(pC_->outString_);

	status = pC_->writeReadController();
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

// function to get limits since done twice and need to read before moving
asynStatus TrinamicAxis::getLimits()
{
    asynStatus status;
    int leftLimit, rightLimit;

	// Read the limit status
	// get lim status: <address> 06 0A/0B (right/left) <motor #> 00 00 00 00 <checksum>
	// get right limit
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x06;
	pC_->outString_[2] = 0x0A;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value (all 0's for read)
	pC_->outString_[4] = 0x00;
	pC_->outString_[5] = 0x00;
	pC_->outString_[6] = 0x00;
	pC_->outString_[7] = 0x00;
		
	pC_->calcTrinamicChecksum(pC_->outString_);
	
	status = pC_->writeReadController();
	if (status) return status;
	// 1 means switch is activated
	rightLimit = (int) (pC_->inString_[7] & 0x000000FF);
	setIntegerParam(pC_->motorStatusHighLimit_, rightLimit);

	// get left limit
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x06;
	pC_->outString_[2] = 0x0B;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value (all 0's for read)
	pC_->outString_[4] = 0x00;
	pC_->outString_[5] = 0x00;
	pC_->outString_[6] = 0x00;
	pC_->outString_[7] = 0x00;
		
	pC_->calcTrinamicChecksum(pC_->outString_);
	
	status = pC_->writeReadController();
	if (status) return status;
	// 1 means switch is activated
	leftLimit = (int) (pC_->inString_[7] & 0x000000FF);
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

	unsigned int accel_int;
    int vel_int;
    // TODO: dont pass pulse and ramp dive to functions, they are controller methods	
	// convert velocity and accel from microsteps to controller units
	vel_int = pC_->vel_steps_to_int(velocity, pC_->pulse_div);	
	accel_int = pC_->accel_steps_to_int(acceleration, pC_->pulse_div, pC_->ramp_div);	

	// set pulse divisor
	// format: <address> 05 9A <motor #> <value (4)> <checksum>
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x05;
	pC_->outString_[2] = 0x9A;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value
	pC_->outString_[4] = (char)((pC_->pulse_div & 0xFF000000) >> 24);
	pC_->outString_[5] = (char)((pC_->pulse_div & 0x00FF0000) >> 16);
	pC_->outString_[6] = (char)((pC_->pulse_div & 0x0000FF00) >> 8);
	pC_->outString_[7] = (char)(pC_->pulse_div & 0x000000FF);		

	// set checksum
	pC_->calcTrinamicChecksum(pC_->outString_);

	status = pC_->writeReadController();
	
	// set ramp divisor
	// format: <address> 05 99 <motor #> <value (4)> <checksum>
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x05;
	pC_->outString_[2] = 0x99;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value
	pC_->outString_[4] = (char)((pC_->ramp_div & 0xFF000000) >> 24);
	pC_->outString_[5] = (char)((pC_->ramp_div & 0x00FF0000) >> 16);
	pC_->outString_[6] = (char)((pC_->ramp_div & 0x0000FF00) >> 8);
	pC_->outString_[7] = (char)(pC_->ramp_div & 0x000000FF);		
	
	pC_->calcTrinamicChecksum(pC_->outString_);

	status = pC_->writeReadController();
	
	// TODO: convert int to char (check if out of char range?)
	// set vel: <address> 05 04 <motor #> <vel (4)> <checksum>
	// TODO: input actual velocity and accel
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x05;
	pC_->outString_[2] = 0x04;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value
	pC_->outString_[4] = (char)((vel_int & 0xFF000000) >> 24);
	pC_->outString_[5] = (char)((vel_int & 0x00FF0000) >> 16);
	pC_->outString_[6] = (char)((vel_int & 0x0000FF00) >> 8);
	pC_->outString_[7] = (char)(vel_int & 0x000000FF);		
	
	pC_->calcTrinamicChecksum(pC_->outString_);

	status = pC_->writeReadController();
	
	// set accl: <address> 05 05 <motor #> <accel (4)> <checksum>
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x05;
	pC_->outString_[2] = 0x05;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value
	pC_->outString_[4] = (char)((accel_int & 0xFF000000) >> 24);
	pC_->outString_[5] = (char)((accel_int & 0x00FF0000) >> 16);
	pC_->outString_[6] = (char)((accel_int & 0x0000FF00) >> 8);
	pC_->outString_[7] = (char)(accel_int & 0x000000FF);		
	
	pC_->calcTrinamicChecksum(pC_->outString_);

	status = pC_->writeReadController();

	return status;
}

asynStatus TrinamicAxis::move(double position, int relative, double minVelocity, double maxVelocity,
		double acceleration)
{
	asynStatus status;

    int leftLimit, rightLimit;
    int targetDir;
    double curPosition;
    
    // TODO
    // separate function for getting limits??
    status = getLimits();
    pC_->getIntegerParam(pC_->motorStatusLowLimit_, &leftLimit);
    pC_->getIntegerParam(pC_->motorStatusHighLimit_, &rightLimit);
    
    pC_->getIntegerParam(pC_->motorStatusDirection_, &targetDir);

	pC_->getDoubleParam(pC_->motorPosition_, &curPosition);

    // // don't send move if limits are activated
    // if ((leftLimit && <dir negative>) || 
    //     (rightLimit && <dir positive>)) 
    //     return asynStatus;

	// set acceleration and velocity:
	status = sendAccelAndVelocity(acceleration, maxVelocity);
	
	// convert position to int
	int pos_int = NINT(position);

	// send move command:
	// TODO	
	if (relative) {
		// move rel: <address> 04 01 <motor #> <rel position (4)> <checksum>
		pC_->outString_[0] = pC_->trinamicAddr;
		pC_->outString_[1] = 0x04;
		pC_->outString_[2] = 0x01;
		pC_->outString_[3] = (char)axisNo_;
	}
	else
	{
		// move abs: <address> 04 00 <motor #> <position (4)> <checksum> 
		pC_->outString_[0] = pC_->trinamicAddr;
		pC_->outString_[1] = 0x04;
		pC_->outString_[2] = 0x00;
		pC_->outString_[3] = (char)axisNo_;
	}
	
	// set 4 bytes of desired value
	pC_->outString_[4] = (char)((pos_int & 0xFF000000) >> 24);
	pC_->outString_[5] = (char)((pos_int & 0x00FF0000) >> 16);
	pC_->outString_[6] = (char)((pos_int & 0x0000FF00) >> 8);
	pC_->outString_[7] = (char)(pos_int & 0x000000FF);		
		
	pC_->calcTrinamicChecksum(pC_->outString_);
	
	status = pC_->writeReadController();
	return status;

}

asynStatus TrinamicAxis::home(double minVelocity, double maxVelocity, 
		double acceleration, int forwards)
{
	asynStatus status;
	// home (aka start reference search RFS): <address> 0D 00 <motor #> <0 (4)> <checksum>

  	status = sendAccelAndVelocity(acceleration, maxVelocity);

	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x0D;
	pC_->outString_[2] = 0x00;
	pC_->outString_[3] = (char)axisNo_;

	pC_->outString_[4] = 0x00;
	pC_->outString_[5] = 0x00;
	pC_->outString_[6] = 0x00;
	pC_->outString_[7] = 0x00;
		
	pC_->calcTrinamicChecksum(pC_->outString_);

  	status = pC_->writeReadController();
  	return status;
}

asynStatus TrinamicAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
	asynStatus status;
	// send accel and velocity
	// double absVelocity = (maxVelocity > 0.) ? maxVelocity : -1*maxVelocity;
	
	// status = sendAccelAndVelocity(acceleration, absVelocity);

	int vel_int = pC_->vel_steps_to_int(maxVelocity, pC_->pulse_div);	
    if (vel_int < 0) vel_int = (-1)*vel_int;
    
	
	// // move vel: <address> 01/02 00 (rotate right/left) <vel position (4)> <checksum> 
	// // maxVelocity can either be positive or negative, so only use rotate right command
	// pC_->outString_[0] = pC_->trinamicAddr;
	// pC_->outString_[1] = 0x01;
	// pC_->outString_[2] = 0x00;
	// pC_->outString_[3] = (char)axisNo_;
	
	if (maxVelocity >= 0) {
		pC_->outString_[0] = pC_->trinamicAddr;
		pC_->outString_[1] = 0x01;
		pC_->outString_[2] = 0x00;
		pC_->outString_[3] = (char)axisNo_;
	}
	else
	{
		pC_->outString_[0] = pC_->trinamicAddr;
		pC_->outString_[1] = 0x02;
		pC_->outString_[2] = 0x00;
		pC_->outString_[3] = (char)axisNo_;
	}
	
	// set 4 bytes of desired value
	pC_->outString_[4] = (char)((vel_int & 0xFF000000) >> 24);
	pC_->outString_[5] = (char)((vel_int & 0x00FF0000) >> 16);
	pC_->outString_[6] = (char)((vel_int & 0x0000FF00) >> 8);
	pC_->outString_[7] = (char)( vel_int & 0x000000FF);		
	
	pC_->calcTrinamicChecksum(pC_->outString_);

    pC_->lock();
	status = pC_->writeReadController();
    epicsThreadSleep(0.01);
    pC_->unlock();

	return status;
}

asynStatus TrinamicAxis::stop(double acceleration)
{
    // TODO: set acceleration?
	asynStatus status;

	// stop: <address> 03 00 <motor #> 00 00 00 00 <checksum>
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x03;
	pC_->outString_[2] = 0x00;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value
	pC_->outString_[4] = 0x00;
	pC_->outString_[5] = 0x00;
	pC_->outString_[6] = 0x00;
	pC_->outString_[7] = 0x00;
		
	pC_->calcTrinamicChecksum(pC_->outString_);

	status = pC_->writeReadController();
	return status;
}

asynStatus TrinamicAxis::setPosition(double position)
{
	asynStatus status;
	
    // send stop command, motor moves when setting position otherwise
    pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x03;
	pC_->outString_[2] = 0x00;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value
	pC_->outString_[4] = 0x00;
	pC_->outString_[5] = 0x00;
	pC_->outString_[6] = 0x00;
	pC_->outString_[7] = 0x00;
		
	pC_->calcTrinamicChecksum(pC_->outString_);

	status = pC_->writeReadController();

	//TODO: NINT + calc position
	int pos_int = NINT(position);
	
	// set pos: <address> 05 01 <motor #> <position (4)> <checksum> 
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x05;
	pC_->outString_[2] = 0x01;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value
	pC_->outString_[4] = (char)((pos_int & 0xFF000000) >> 24);
	pC_->outString_[5] = (char)((pos_int & 0x00FF0000) >> 16);
	pC_->outString_[6] = (char)((pos_int & 0x0000FF00) >> 8);
	pC_->outString_[7] = (char)(pos_int & 0x000000FF);		
		
	pC_->calcTrinamicChecksum(pC_->outString_);

	status = pC_->writeReadController();
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
	int rightLimit;
	int leftLimit;
	double position;
	asynStatus comStatus;

	// Read the current motor position
	// get pos: <address> 06 01 <motor #> 00 00 00 00 <checksum> 
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x06;
	pC_->outString_[2] = 0x01;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value (all 0's for read)
	pC_->outString_[4] = 0x00;
	pC_->outString_[5] = 0x00;
	pC_->outString_[6] = 0x00;
	pC_->outString_[7] = 0x00;
		
	pC_->calcTrinamicChecksum(pC_->outString_);
	
	comStatus = pC_->writeReadController();
	if (comStatus) goto skip;
	// value of a read is in bytes 4-7	
	// response placed in pC_->inString_
	// convert 8 bytes to double and set parameter
	position = (double) (int)(
		((pC_->inString_[4] << 24) & 0xFF000000) | 
		((pC_->inString_[5] << 16) & 0x00FF0000) | 
		((pC_->inString_[6] << 8) & 0x0000FF00) | 
		((pC_->inString_[7] << 0) & 0x000000FF) );

	setDoubleParam(pC_->motorPosition_, position);

    // check if velocity != 0
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x06;
	pC_->outString_[2] = 0x03;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value (all 0's for read)
	pC_->outString_[4] = 0x00;
	pC_->outString_[5] = 0x00;
	pC_->outString_[6] = 0x00;
	pC_->outString_[7] = 0x00;
	
    pC_->calcTrinamicChecksum(pC_->outString_);
	
	comStatus = pC_->writeReadController();
	if (comStatus) goto skip;
	
    // check if any byte is not 0
    done = !(pC_->inString_[4] || pC_->inString_[5] || pC_->inString_[6] || pC_->inString_[7]);
    // if ((pC_->inString_[4] != 0)||
    //     (pC_->inString_[5] != 0)||
    //     (pC_->inString_[6] != 0)||
    //     (pC_->inString_[7] != 0))
    // {
    //     done=0;
    // }
    // else {
    //     done=1;
    // } 

	setIntegerParam(pC_->motorStatusDone_, done);
	*moving = done ? false : true;	

	// // Read the moving status of this motor
	// // get moving status (position reached flag, 0 if moving): 
	// // <address> 06 08 <motor #> 00 00 00 00 <checksum>
	// pC_->outString_[0] = pC_->trinamicAddr;
	// pC_->outString_[1] = 0x06;
	// pC_->outString_[2] = 0x08;
	// pC_->outString_[3] = (char)axisNo_;

	// // set 4 bytes of desired value (all 0's for read)
	// pC_->outString_[4] = 0x00;
	// pC_->outString_[5] = 0x00;
	// pC_->outString_[6] = 0x00;
	// pC_->outString_[7] = 0x00;
	//  
	// pC_->calcTrinamicChecksum(pC_->outString_);
	// 
	// comStatus = pC_->writeReadController();
	// if (comStatus) goto skip;
	// // only need LSB for motor flag
	// // 1 means position reached
	// done = (int) (pC_->inString_[7] & 0x000000FF);

	// setIntegerParam(pC_->motorStatusDone_, done);
	// *moving = done ? false : true;	

	// Read the limit status
	// get lim status: <address> 06 0A/0B (right/left) <motor #> 00 00 00 00 <checksum>
	// get right limit
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x06;
	pC_->outString_[2] = 0x0A;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value (all 0's for read)
	pC_->outString_[4] = 0x00;
	pC_->outString_[5] = 0x00;
	pC_->outString_[6] = 0x00;
	pC_->outString_[7] = 0x00;
		
	pC_->calcTrinamicChecksum(pC_->outString_);
	
	comStatus = pC_->writeReadController();
	if (comStatus) goto skip;
	// 1 means switch is activated
	rightLimit = (int) (pC_->inString_[7] & 0x000000FF);
	setIntegerParam(pC_->motorStatusHighLimit_, rightLimit);

	// get left limit
	pC_->outString_[0] = pC_->trinamicAddr;
	pC_->outString_[1] = 0x06;
	pC_->outString_[2] = 0x0B;
	pC_->outString_[3] = (char)axisNo_;

	// set 4 bytes of desired value (all 0's for read)
	pC_->outString_[4] = 0x00;
	pC_->outString_[5] = 0x00;
	pC_->outString_[6] = 0x00;
	pC_->outString_[7] = 0x00;
		
	pC_->calcTrinamicChecksum(pC_->outString_);
	
	comStatus = pC_->writeReadController();
	if (comStatus) goto skip;
	// 1 means switch is activated
	leftLimit = (int) (pC_->inString_[7] & 0x000000FF);
	setIntegerParam(pC_->motorStatusLowLimit_, leftLimit);

    // cancel move (stop) if limit hit
    // default behavior is that if limit is lifted, move will continue
    // if ((rightLimit || leftLimit) && !(pastLeftLimit || pastRightLimit)) comStatus = stop(0);
    if ((rightLimit && !pastRightLimit) || (leftLimit && !pastLeftLimit)) comStatus = stop(0);
    pastLeftLimit = leftLimit;
    pastRightLimit = rightLimit;

	skip:
	setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
	callParamCallbacks();
	return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg TrinamicCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg TrinamicCreateControllerArg1 = {"Trinamic port name", iocshArgString};
static const iocshArg TrinamicCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg TrinamicCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg TrinamicCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg TrinamicCreateControllerArg5 = {"pulse divisor", iocshArgInt};
static const iocshArg TrinamicCreateControllerArg6 = {"ramp divisor", iocshArgInt};
static const iocshArg TrinamicCreateControllerArg7 = {"run current", iocshArgInt};
static const iocshArg TrinamicCreateControllerArg8 = {"standby current", iocshArgInt};
static const iocshArg TrinamicCreateControllerArg9 = {"microstep resolution", iocshArgInt};
static const iocshArg * const TrinamicCreateControllerArgs[] = {&TrinamicCreateControllerArg0,
                                                             &TrinamicCreateControllerArg1,
                                                             &TrinamicCreateControllerArg2,
                                                             &TrinamicCreateControllerArg3,
                                                             &TrinamicCreateControllerArg4,
                                                             &TrinamicCreateControllerArg5,
                                                             &TrinamicCreateControllerArg6,
                                                             &TrinamicCreateControllerArg7,
                                                             &TrinamicCreateControllerArg8,
                                                             &TrinamicCreateControllerArg9};
static const iocshFuncDef TrinamicCreateControllerDef = {"TrinamicCreateController", 10, 
	TrinamicCreateControllerArgs};

static void TrinamicCreateControllerCallFunc(const iocshArgBuf *args)
{
	TrinamicCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, 
        args[4].ival, args[5].ival, args[6].ival, args[7].ival, args[8].ival, args[9].ival);
}

static void TrinamicRegister(void)
{
	iocshRegister(&TrinamicCreateControllerDef, TrinamicCreateControllerCallFunc);
}

extern "C" {
	epicsExportRegistrar(TrinamicRegister);
}
