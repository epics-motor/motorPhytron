/*
SPDX-License-Identifier: EPICS
FILENAME... phytronAxisMotor.h
USAGE...    Motor record  support for Phytron Axis controller.

Tom Slejko & Bor Marolt
Cosylab d.d. 2014

Lutz Rossa, Helmholtz-Zentrum Berlin fuer Materialien und Energy GmbH, 2021-2025

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include <initHooks.h>


//Number of controller specific parameters
#define NUM_PHYTRON_PARAMS 33

#define MAX_VELOCITY      500000 //steps/s
#define MIN_VELOCITY      1      //steps/s

#define MAX_ACCELERATION  500000  // steps/s^2
#define MIN_ACCELERATION  4000    // steps/s^2

//Controller parameters
#define controllerStatusString      "CONTROLLER_STATUS"
#define controllerStatusResetString "CONTROLLER_STATUS_RESET"
#define resetControllerString       "CONTROLLER_RESET"

//Axis parameters
#define axisStatusString            "AXIS_STATUS"
#define homingProcedureString       "HOMING_PROCEDURE"
#define axisModeString              "AXIS_MODE"
#define mopOffsetPosString          "MOP_POS"
#define mopOffsetNegString          "MOP_NEG"
#define stepResolutionString        "STEP_RES"
#define stopCurrentString           "STOP_CURRENT"
#define runCurrentString            "RUN_CURRENT"
#define boostCurrentString          "BOOST_CURRENT"
#define encoderTypeString           "ENCODER_TYP"
#define initRecoveryTimeString      "INIT_TIME"
#define positionRecoveryTimeString  "POSITION_TIME"
#define boostConditionString        "BOOST"
#define encoderRateString           "ENC_RATE"
#define switchTypString             "SWITCH_TYP"
#define pwrStageModeString          "PWR_STAGE_MODE"
#define encoderResolutionString     "ENC_RESOLUTION"
#define encoderFunctionString       "ENC_FUNCTION"
#define encoderSFIWidthString       "ENC_SFI_WIDTH"
#define encoderDirectionString      "ENC_DIRECTION"
#define powerStageTempString        "PS_TEMPERATURE"
#define powerStagetMonitorString    "PS_MONITOR"
#define motorTempString             "MOTOR_TEMP"
#define currentDelayTimeString      "CURRENT_DELAY_TIME"
#define axisResetString             "AXIS_RESET"
#define axisStatusResetString       "AXIS_STATUS_RESET"
#define axisBrakeOutputString       "AXIS_BRAKE_OUTPUT"
#define axisDisableMotorString      "AXIS_DISABLE_MOTOR"
#define axisBrakeEngageTimeString   "AXIS_BRAKE_ENGAGE_TIME"
#define axisBrakeReleaseTimeString  "AXIS_BRAKE_RELEASE_TIME"
#define directCommandString         "DIRECT_COMMAND"
#define directReplyString           "DIRECT_REPLY"
#define directStatusString          "DIRECT_STATUS"

typedef enum {
  phytronSuccess,
  phytronTimeout,
  phytronOverflow,
  phytronError,
  phytronDisconnected,
  phytronDisabled,
  phytronInvalidReturn,
  phytronInvalidCommand
} phytronStatus;

enum movementType{
  stdMove,
  homeMove,
  stopMove
};

enum homingType{
  limit,
  center,
  encoder,
  limitEncoder,
  centerEncoder,
  referenceCenter,
  referenceCenterEncoder,
};

enum pollMethod{
  pollMethodDefault            = -1, /// only for axis: use controller setting
  pollMethodSerial             =  0, /// old/default: 4 times single command+reply
  pollMethodAxisParallel       =  1, /// axis-parallel: combine 3 commands/replies into one string
  pollMethodControllerParallel =  2  /// controller-parallel: combine up to 32 axes a 3 commands/replies into one string
};

class phytronController;
class phytronIO;
class phytronAxis : public asynMotorAxis
{
public:
  // common asyn functions
  phytronAxis(class phytronController *pC, int iAxisNo);
  void report(FILE *fp, int level);

  // motor record functions
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setEncoderRatio(double ratio);
  asynStatus setEncoderPosition(double position);

  // phytron axis functions
  static int phytronCreateAxis(const char* szControllerName, int iModule, int iAxis);
  static int phytronBrakeOutput(const char* szControllerName, float fAxis, float fOutput, int bDisableMotor, double dEngageTime, double dReleaseTime);
  asynStatus configureBrake(float fOutput, bool bDisableMotor, double dEngageTime, double dReleaseTime);

  char  axisModuleNo_[5];         ///<Used by sprintf to form commands
  float brakeOutput_;             ///<<module>.<index> of digital output to drive for brake (or -1)
  int   disableMotor_;            ///<bit0: 0=keep motor enabled, 1=disable idle motor/enable when moved, bit1: 0=disabled, 1=enabled
  float brakeEngageTime_;         ///<time to engage brake (disable motor after this time in milliseconds)
  float brakeReleaseTime_;        ///<time to release brake (start move after this time in milliseconds)

protected:
  void appendRequestString(std::vector<std::string> &asCommands) const;
  bool parseAnswer(std::vector<std::string> &asValues);

private:
  phytronController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */

  phytronStatus clearAxisError(std::vector<std::string>* pCmdList);
  phytronStatus setVelocity(std::vector<std::string>* pCmdList, double minVelocity, double maxVelocity, int moveType);
  phytronStatus setAcceleration(std::vector<std::string>* pCmdList, double acceleration, int movementType);
  phytronStatus setBrakeOutput(std::vector<std::string>* pCmdList, bool bWantToMoveMotor);

  phytronStatus lastStatus_;    ///< last Phytron status
  int brakeReleased_;           ///< state of brake
  enum pollMethod iPollMethod_; ///< individual poll method for this axis
  int homeState_;               ///< state machine for work around homing to limit switches

friend class phytronController;
};

class phytronController : public asynMotorController
{
public:
  enum TYPE { TYPE_PHYMOTION, TYPE_MCC };

  // common asyn functions
  phytronController(TYPE iCtrlType, const char* szPhytronPortName, const char* szAsynPortName, int iAddress, double iMovingPollPeriod, double iIdlePollPeriod, double dTimeout, bool iNoResetAtBoot);
  asynStatus readInt32(asynUser* pasynUser, epicsInt32* piValue);
  asynStatus writeInt32(asynUser* pasynUser, epicsInt32 iValue);
  asynStatus readFloat64(asynUser* pasynUser, epicsFloat64* pdValue);
  asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 dValue);
  asynStatus writeOctet(asynUser* pasynUser, const char* szValue, size_t maxChars, size_t* pnActual);
  asynStatus readOption(asynUser* pasynUser, const char* szKey, char* szValue, int iMaxChars);
  asynStatus writeOption(asynUser* pasynUser, const char* szKey, const char* szValue);
  void report(FILE *fp, int level);

  // motor record functions
  phytronAxis* getAxis(asynUser *pasynUser);
  phytronAxis* getAxis(int axisNo);
  asynStatus poll();

  // phytron controller functions
  static int phytronCreatePhymotion(const char* szPhytronPortName, const char* szAsynPortName, int iMovingPollPeriod, int iIdlePollPeriod, double dTimeout, int iNoResetAtBoot);
  static int phytronCreateMCC(const char* szPhytronPortName, const char* szAsynPortName, int iAddress, int iMovingPollPeriod, int iIdlePollPeriod, double dTimeout, int iNoResetAtBoot);
  phytronStatus sendPhytronCommand(std::string sCommand, std::string &sResponse, bool bACKonly = true);
  phytronStatus sendPhytronCommand(std::string sCommand, std::string* psResponse = nullptr, bool bACKonly = true);
  phytronStatus sendPhytronMultiCommand(std::vector<std::string> asCommands, std::vector<std::string> &asResponses, bool bAllowNAK = false, bool bForceSingle = false);
  void resetAxisEncoderRatio();

  //casts phytronStatus to asynStatus
  static asynStatus phyToAsyn(phytronStatus phyStatus);
  static std::string escapeC(std::string sIn);

  char * controllerName_;
  std::vector<phytronAxis*> axes_;
  std::vector<phytronIO*> IOs_;
  enum TYPE iCtrlType_;                ///< controller type

protected:
  //Additional parameters used by additional records
  int axisStatus_;
  int controllerStatus_;
  int homingProcedure_;
  int axisMode_;
  int mopOffsetPos_;
  int mopOffsetNeg_;
  int stepResolution_;
  int stopCurrent_;
  int runCurrent_;
  int boostCurrent_;
  int encoderType_;
  int initRecoveryTime_;
  int positionRecoveryTime_;
  int boost_;
  int encoderRate_;
  int switchTyp_;
  int pwrStageMode_;
  int encoderRes_;
  int encoderFunc_;
  int encoderSFIWidth_;
  int encoderDirection_;
  int powerStageTemp_;
  int powerStageMonitor_;
  int motorTemp_;
  int currentDelayTime_;
  int resetController_;
  int axisReset_;
  int axisStatusReset_;
  int controllerStatusReset_;
  int axisBrakeOutput_;
  int axisDisableMotor_;
  int axisBrakeEngageTime_;
  int axisBrakeReleaseTime_;
  int directCommand_;
  int directReply_;
  int directStatus_;

private:
  static std::vector<phytronController*> controllers_; ///< list of all phytron controllers
  int    iAddress_;                    ///< serial address of controller
  double timeout_;                     ///< communication timeout
  phytronStatus lastStatus_;           ///< last communication status
  bool do_initial_readout_;            ///< helper for initIOC hook to wait for first poll
  enum pollMethod iDefaultPollMethod_; ///< default poll method for every axis
  bool fake_homed_enable_;             ///< enable fake HOMED bits with help Phytron registers 1001...1020 (max. 20 module positions)
  epicsUInt16 fake_homed_cache_[20];   ///< cached value, which is updated with every poll (max. 20 module positions)
  bool allow_exit_on_error_;           ///< allow exit(1) on error
  std::string sLastSUI_;               ///< last response to SUI command (MCC only)
  std::string sCtrlType_;              ///< controller type

  static void epicsInithookFunction(initHookState iState);

friend class phytronAxis;
friend class phytronIO;
};
