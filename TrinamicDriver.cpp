/*

FILENAME... TrinamicDriver.cpp
USAGE...    Motor driver support for any Trinamic controller that uses the TMCL language.
	Note that this must be in ASCII mode and not binary mode.

Bryan Monk
March 14, 2023

Based on https://github.com/epics-motor/motorAcs/blob/master/acsApp/src/TrinamicDriver.cpp
	by Mark Rivers

*/

#include <stdio.h>
#include <stdlib.h>

#include <TrinamicDriver.h>

#include <asynOctetSyncIO.h>

#include <epicsExport.h>
#include <epicsThread.h>

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

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
	// Read the limit status
	// Read the drive power on status (?)


	// TODO: what this do???
	skip:
	setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
	callParamCallbacks();
	return comStatus ? asynError : asynSuccess;
}
