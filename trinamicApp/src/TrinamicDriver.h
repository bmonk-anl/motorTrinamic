// bug report:
// setting position: need to stop before doing it, says moving after
// use velocity instead of position reached flag

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_TRINAMIC_AXES 6
#define NUM_TRINAMIC_PARAMS 0 

// fixed # of bytes that is sent with each command 
#define TRINAMIC_CMD_SIZE 9

#define TRINAMIC_ADDR 0
#define DEFAULT_PULSE_DIV 3
#define DEFAULT_RAMP_DIV 7
#define DEFAULT_RUN_CURRENT 127 
#define DEFAULT_STANDBY_CURRENT 8
#define DEFAULT_USTEP_RES 8 


class epicsShareClass TrinamicAxis : public asynMotorAxis
{
	public:
		/* These are the methods we override from the base class */
		TrinamicAxis(class TrinamicController* pC, int axis);
		void report(FILE* fp, int level);
		asynStatus move(double position, int relative, double min_velocity, double max_velocity, 
				double acceleration);
		asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
		asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
		asynStatus stop(double acceleration);
		asynStatus poll(bool* moving);
		asynStatus setPosition(double position);
		//asynStatus setClosedLoop(bool closedLoop);
	private:
		TrinamicController* pC_;

        int pastLeftLimit=0;
        int pastRightLimit=0;
		//functions below for controller specific commands
		asynStatus sendAccelAndVelocity(double accel, double velocity);
	friend class TrinamicController;
};

class epicsShareClass TrinamicController : public asynMotorController 
{
	public:
		TrinamicController(const char* portName, const char* TrinamicPortName, int numAxes, 
				double movingPollPeriod, double idlePollPeriod, unsigned int pulse_div, 
                unsigned int ramp_div, unsigned int run_current, unsigned int standby_current, 
                unsigned int ustep_res);
		
		void report(FILE* fp, int level);
		TrinamicAxis* getAxis(asynUser *pasynUser);
		TrinamicAxis* getAxis(int axisNo);

		// need to rewrite methods since raw binary is used in commands and this
		// method uses strlen (will be wrong if data contains byte of 0)
		asynStatus writeReadController();
		asynStatus writeReadController(const char *output, char *input, 
                                                    size_t maxChars, size_t *nread, double timeout);
		void calcTrinamicChecksum(char* command);
		int vel_steps_to_int (double velocity, unsigned int pulse_div);
		unsigned int accel_steps_to_int (double acceleration, unsigned int pulse_div, 
			unsigned int ramp_div);

		char trinamicAddr = TRINAMIC_ADDR;
        // TODO: set below to chars?
		unsigned int pulse_div = DEFAULT_PULSE_DIV;
		unsigned int ramp_div = DEFAULT_RAMP_DIV;
        unsigned int run_current = DEFAULT_RUN_CURRENT; 
        unsigned int standby_current = DEFAULT_STANDBY_CURRENT; 
        unsigned int ustep_res = DEFAULT_USTEP_RES; 

	friend class TrinamicAxis;
};
									
