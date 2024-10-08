// TODO: 
// ADD ARG FOR LIMIT POLARITY
// make sub file with diff ustep res and show max speeds
// why homing accel weird? if homing is first thing done, might cause problems
// figure out which methods to return after each failed command 
// add additional parameters to initialization print
// still get bug of not stopping
// cap position sent?
// send message when send too high of vel or pos
// check for ranges in createcontroller params
// homing really messed up - too fast of speed
//
//
// add asyn params instead of sending info on startup:
// pulse div
// ramp div
// run current
// standby current
// microstep resolution
//
// make per axis pulse/ramp/ustep ?

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_TRINAMIC_AXES 6
// number of asyn params
// #define NUM_TRINAMIC_PARAMS 6

// fixed # of bytes that is sent with each command 
#define TRINAMIC_CMD_SIZE 9

#define TRINAMIC_ADDR 1
#define DEFAULT_PULSE_DIV 3
#define DEFAULT_RAMP_DIV 7
#define DEFAULT_RUN_CURRENT 127 
#define DEFAULT_STANDBY_CURRENT 8
#define DEFAULT_USTEP_RES 8 
#define DEFAULT_LIM_MASK 0x0000

// ASYN PARAMS
// each parameter will have one per axis, below is a base string combined with axis number 
// in the createParam function

#define PulseDivString_BASE "PULSE_DIV"
#define RampDivString_BASE "RAMP_DIV"
#define RunCurrentString_BASE "RUN_CURRENT"
#define StandbyCurrentString_BASE "STANDBY_CURRENT"
#define UStepResString_BASE "USTEP_RES"
#define PulseDivRBVString_BASE "PULSE_DIV_RBV"
#define RampDivRBVString_BASE "RAMP_DIV_RBV"
#define RunCurrentRBVString_BASE "RUN_CURRENT_RBV"
#define StandbyCurrentRBVString_BASE "STANDBY_CURRENT_RBV"
#define UStepResRBVString_BASE "USTEP_RES_RBV"

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
    private:
        TrinamicController* pC_;
        
        //functions below for controller specific commands
        asynStatus sendAccelAndVelocity(double accel, double velocity);
        asynStatus getLimits();
        int homingInProg = 0;
        
    friend class TrinamicController;
};

class epicsShareClass TrinamicController : public asynMotorController 
{
    public:
        TrinamicController(const char* portName, const char* TrinamicPortName, int numAxes, 
            double movingPollPeriod, double idlePollPeriod, 
            // unsigned int pulse_div, 
            // unsigned int ramp_div, unsigned int run_current, unsigned int standby_current, 
            // unsigned int ustep_res, 
            char module_addr, const char* model, int limMask);
        
        void report(FILE* fp, int level);
        TrinamicAxis* getAxis(asynUser *pasynUser);
        TrinamicAxis* getAxis(int axisNo);
        
        asynStatus writeReadController();
        asynStatus writeReadController(const char *output, char *input, 
                        size_t maxChars, size_t *nread, double timeout);
        asynStatus sendIntTMCL(char arg0, char arg1, char arg2, char arg3, int val);
        void calcTrinamicChecksum(char* command);
        int vel_steps_to_int (double velocity, unsigned int pulse_div);
        unsigned int accel_steps_to_int (double acceleration, unsigned int pulse_div, 
		unsigned int ramp_div);

        // asynportdriver function for writing extra params:
        virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

        // member variables to be set by asyn functions
        // unsigned int pulse_div = DEFAULT_PULSE_DIV;
        // unsigned int ramp_div = DEFAULT_RAMP_DIV;
        // unsigned int run_current = DEFAULT_RUN_CURRENT; 
        // unsigned int standby_current = DEFAULT_STANDBY_CURRENT; 
        // unsigned int ustep_res = DEFAULT_USTEP_RES; 

        int limMask = DEFAULT_LIM_MASK;

        std::string model = "6214"; // default model

        int numAxes;
        
        char module_addr = TRINAMIC_ADDR;
    protected:
        int PulseDiv_[6];
        int RampDiv_[6];
        int RunCurrent_[6];
        int StandbyCurrent_[6];
        int UStepRes_[6];

        int PulseDiv_RBV_[6];
        int RampDiv_RBV_[6];
        int RunCurrent_RBV_[6];
        int StandbyCurrent_RBV_[6];
        int UStepRes_RBV_[6];

	#define FIRST_TRINAMIC_PARAM PulseDiv_[0]
	#define LAST_TRINAMIC_PARAM UStepRes_RBV_[5]

    private:
        asynStatus setPulseDiv(epicsInt32 value, int axis);
        asynStatus setRampDiv(epicsInt32 value, int axis);
        asynStatus setRunCurrent(epicsInt32 value, int axis);
        asynStatus setStandbyCurrent(epicsInt32 value, int axis);
        asynStatus setUStepRes(epicsInt32 value, int axis);
        
    friend class TrinamicAxis;
};
									
#define NUM_TRINAMIC_PARAMS ((int)(&LAST_TRINAMIC_PARAM - &FIRST_TRINAMIC_PARAM + 1))

