#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_TRINAMIC_AXES 6

// No controller-specific parameters yet
#define NUM_TRINAMIC_PARAMS 0 

class epicsShareClass TrnamicAxis : public asynMotorAxis
{
	public:
		/* These are the methods we override from the base class */
		TranamicAxis(class TrinamicController* pC, int axis);
		void report(FILE* fp, int level);
		asynStatus move(double position, int relative, double min_velocity, double max_velocity, 
				double acceleration);
		asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
		//asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
		asynStatus stop(double acceleration);
		asynStatus poll(bool* moving);
		asynStatus setPosition(double position);
		//asynStatus setClosedLoop(bool closedLoop);
	private:
		TrinamicController* pC_;
		//functions below for controller specific commands
		asynStatus sendAccelAndVelocity(double accel, double velocity);
	friend class TrinamicController;
};

class epicsShareClass TrinamicController : public asynMotorController 
{
	public:
		TrinamicController(const char* portName, const char* TrinamicPortName, int numAxes, 
				double movingPollPeriod, double idlePollPeriod);
		
		void report(FILE* fp, int level);
		TrinamicAxis* getAxis(asynUser *pasynUser);
		TrinamicAxis* getAxis(int axisNo);

	friend class TrinamicAxis;
}
									
