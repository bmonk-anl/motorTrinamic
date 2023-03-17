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

#include <epicsExport.h>
#include <epicsThread.h>

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

// set checksum directly in function or return number?
void calcTrinamicChecksum(uint8_t* command)
{
	uint8_t checksum, i;

	for(i=0; i<=8; i++) {
		checksum += command[i];	
	}

	command[8] = checksum;
	//return checksum; 
}


/** Creates a new TrinamicController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] TrinamicPortName     The name of the drvAsynSerialPort that was created previously to connect to the Trinamic controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */

TrinamicController::TrinamicController(const char* portName, const char* TrinamicPortName,
		int numAxes, double movingPollPeriod, double idlePollPeriod)
	: asynMotorController(portName, numAxes, NUM_TRINAMIC_PARAMS, 
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
	int axis;
	asynStatus status;
	TrinamicAxis* pAxis;
	static const char* functionName = "TrinamicController::TrinamicController";

	// Connect to Trinamic Controller
	status = pasynOctetIO->connect(TrinamicPortName, 0, &pasynUserController_, NULL);
	if(status) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
				"%s: cannot connect to Trinamic controller\n", functionName);
	}
	for(axis=0; axis<numAxis; axis++)
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

extern "C" int CreatTrinamicController(const* char portName, const char* TrinamicPortName, 
		int numAxes, int movingPollPeriod, idlePollPeriod)
{
	TrinamicController* pTrinamicController = new TrinamicController(portName, TrinamicPortName,
			numAxes, movingPollPeriod/1000., idlePollPeriod/1000.,);
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


asynStatus TrinamicAxis::sendAccelAndVelocity(double acceleration, double velocity)
{
	asynStatus status;
	int pulse_div, ramp_div;
	
	// get pulse divisor
	// TODO: what happens if no response?
	sprintf(pC_->outString_, "TODO", axisNo_);
	comStatus = pC_->writeReadController();
	if (comStatus) goto skip;
	// the response is in the form TODO
	pulse_div = std::stoi(&pC_->inString_[5]);
	
	// get ramp divisor
	sprintf(pC_->outString_, "TODO", axisNo_);
	comStatus = pC_->writeReadController();
	if (comStatus) goto skip;
	// the response is in the form TODO
	ramp_div = std::stoi(&pC_->inString_[5]);

	// calculate velocity - convert microsteps to internal units
	int v_int = NINT(velocity)*2

	// send velocity
	sprintf(pC->outString_, "TODO", axisNo_, todo);
	status = pC_->writeReadController();
	return status;

	// send acceleration
	sprintf(pC->outString_, "TODO", axisNo_, todo);
	status = pC_->writeReadController();
	return status;
	
	skip:
	return asynError;

}

asynStatus TrinamicAxis::move(double position, int relative, double minVelocity, double maxVelocity,
		double acceleration))
{
	asynStatus status;

	// set acceleration and velocity:
	// TODO
	// sprintf(pC->outString_, "TODO", axisNo_, ival);
	// status = pC_->writeReadController();
	
	// send move command:
	// TODO	
	if (relative) {
		sprintf(pC->outString_, "TODO", axisNo_, NINT(position));
	}
	else
	{
		sprintf(pC->outString_, "TODO", axisNo_, NINT(position));
	}
	
	status = pC_->writeReadController();
	return status;

}

asynStatus TrinamicAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
	asynStatus status;
	// TODO: send accel and velocity
	
	if (maxVelocity > 0.) {
		//TODO	
		sprintf(pC->outString_, "TODO", axisNo_);
	}
	else
	{
		//TODO
		sprintf(pC->outString_, "TODO", axisNo_);
	}
	status = pC_->writeReadController();
	return status;
}

asynStatus TranamicAxis::stop(double acceleration)
{
	asynStatus status;

	sprintf(pC_->outString_, "TODO", axisNo_);

	status = pC_->writeReadController();
	return status;
}

asynStatus Trinamic::setPosition(double position)
{
	asynStatus status;

	//TODO: NINT?
	sprintf(pC_->outString_, "TODO", axisNo_, position);

	status = pC_->writeReadController();
	return status;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status,
  * and the drive power-on status.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */

asynStatus TrinamicAxis::poll(bool *moving)
{
	int done;
	int driveOn;
	int limit;
	double position;
	asynStatus comStatus;

	// TODO: what is comstatus and why skip?

	// Read the current motor position
	sprintf(pC_->outString_, "TODO", axisNo_);
	comStatus = pC_->writeReadController();
	if (comStatus) goto skip;
	// the response is in the form TODO
	position = atof(&pC_->inString_[5]);
	setDoubleParam(pC_->motorPosition_, position);

	// Read the moving status of this motor
	sprintf(pC_->outString_, "TODO", axisNo_);
	comStatus = pC_->writeReadController();
	if (comStatus) goto skip;
	// the response is in the form TODO
	done = (pC_->inString_[5] == '0') ? 1:0;
	setIntegerParam(pC_->motorStatusDone_, done);
	*moving = done ? false:true;	

	// Read the limit status
	sprintf(pC_->outString_, "TODO", axisNo_);
	comStatus = pC_->writeReadController();
	if (comStatus) goto skip;
	// the response is in the form TODO
	limit = (pC_->inString_[5] == '1') ? 1:0;
	setIntegerParam(pC_->motorStatusHighLimit_, limit);
	limit = (pC_->inString_[6] == '1') ? 1:0;
	setIntegerParam(pC_->motorStatusLowLimit_, limit);
	limit = (pC_->inString_[7] == '1') ? 1:0;
	setIntegerParam(pC_->motorStatusAtHome_, limit);

	// Read the drive power on status (?)
	sprintf(pC_->outString_, "TODO", axisNo_);
	comStatus = pC_->writeReadController();
	if (comStatus) goto skip;
	// the response is in the form TODO
	driveOn = (pC_->inString_[5] == '1') ? 1:0;
	setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
	setIntegerParam(pC_->motorStatusProblem_, 0);

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
static const iocshArg * const TrinamicCreateControllerArgs[] = {&TrinamicCreateControllerArg0,
                                                             &TrinamicCreateControllerArg1,
                                                             &TrinamicCreateControllerArg2,
                                                             &TrinamicCreateControllerArg3,
                                                             &TrinamicCreateControllerArg4};
static const iocshFuncDef TrinamicCreateControllerDef = {"TrinamicCreateController", 5, 
	TrinamicCreateControllerArgs};

static void TrinamicCreateContollerCallFunc(const iocshArgBuf *args)
{
	TrinamicCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void TrinamicRegister(void)
{
	iocshRegister(&TrinamicCreateControllerDef, TrinamicCreateContollerCallFunc);
}

extern "C" {
	epicsExportRegistrar(TrinamicRegister);
}
