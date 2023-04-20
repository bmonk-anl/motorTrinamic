#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_TRINAMIC_AXES 6
#define NUM_TRINAMIC_PARAMS 0 

// fixed # of bytes that is sent with each command 
#define TRINAMIC_CMD_SIZE 9

#define TRINAMIC_ADDR 0
#define PULSE_DIV 3
#define RAMP_DIV 7


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

		// need to rewrite methods since raw binary is used in commands and this
		// method uses strlen (will be wrong if data contains byte of 0)
		asynStatus writeReadController()
		asynStatus writeReadController(const char *output, char *input, 
                                                    size_t maxChars, size_t *nread, double timeout)

		char trinamicAddr = TRINAMIC_ADDR;
		unsigned int pulse_div = PULSE_DIV;
		unsigned int ramp_div = RAMP_DIV;

	friend class TrinamicAxis;
}
									
