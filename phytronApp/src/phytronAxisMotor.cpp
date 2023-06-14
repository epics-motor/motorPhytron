/*
FILENAME... phytronAxisMotor.cpp
USAGE...    Motor driver support for Phytron Axis controller.

Tom Slejko & Bor Marolt
Cosylab d.d. 2014

Lutz Rossa, Helmholtz-Zentrum Berlin fuer Materialien und Energy GmbH, 2021-2023

*/

#include <stdio.h>
#include <stdint.h>
#include <algorithm>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#ifndef _WIN32
#include <unistd.h>
#endif

#include <drvAsynIPPort.h>
#include <iocsh.h>
#include <epicsThread.h>
#include <epicsStdlib.h>
#include <epicsString.h>
#include <cantProceed.h>

#include <asynOctetSyncIO.h>

#include "phytronAxisMotor.h"
#include <epicsExport.h>

using namespace std;

#ifndef ASYN_TRACE_WARNING
#define ASYN_TRACE_WARNING ASYN_TRACE_ERROR
#endif

#define CHECK_CTRL(xfunc, xtxt, xextra) \
    if (phyStatus) { \
      if (phyStatus != lastStatus) { \
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, \
          "phytronController::%s: (%s) %s failed with error code: %d, reason: %d\n", \
          xfunc, this->controllerName_, xtxt, phyStatus, pasynUser->reason); \
        lastStatus = phyStatus; \
      } \
      xextra; \
    } \
    lastStatus = phyStatus

#define CHECK_AXIS(xfunc, xtxt, xinstance, xextra) \
    if (phyStatus) { \
      if (phyStatus != lastStatus) { \
        asynPrint(xinstance->pasynUser_, ASYN_TRACE_ERROR, \
          "phytronAxis::%s: (%s) %s for axis %d failed with error code: %d\n", \
          xfunc, xinstance->pC_->controllerName_, xtxt, xinstance->axisNo_, phyStatus); \
        lastStatus = phyStatus; \
      } \
      xextra; \
    } \
    lastStatus = phyStatus

//Used for casting position doubles to integers
#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

//Specify maximum sleep time after brake was (de)activated in seconds
#define MAXIMUM_BRAKE_TIME 10.0

/*
 * Contains phytronController instances, phytronCreateAxis uses it to find and
 * bind axis object to the correct controller object.
 */
static vector<phytronController*> controllers;

/** Creates a new phytronController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] phytronPortName   The name of the drvAsynIPPort that was created previously to connect to the phytron controller
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
phytronController::phytronController(const char *phytronPortName, const char *asynPortName,
                                     double movingPollPeriod, double idlePollPeriod, double timeout)
  :  asynMotorController(phytronPortName,
                         0xFF,
                         NUM_PHYTRON_PARAMS,
                         asynOptionMask, // additional interfaces
                         0, //No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)// Default priority and stack size
  , do_initial_readout_(true)
  , iDefaultPollMethod_(pollMethodSerial)
{
  asynStatus status;
  static const char *functionName = "phytronController::phytronController";

  //Timeout is defined in milliseconds, but sendPhytronCommand expects seconds
  timeout_ = timeout/1000;

  //pyhtronCreateAxis uses portName to identify the controller
  this->controllerName_ = (char *) mallocMustSucceed(sizeof(char)*(strlen(portName)+1),
      "phytronController::phytronController: Controller name memory allocation failed.\n");

  strcpy(this->controllerName_, portName);

  //Create Controller parameters
  createParam(controllerStatusString,     asynParamInt32, &this->controllerStatus_);
  createParam(controllerStatusResetString,asynParamInt32, &this->controllerStatusReset_);
  createParam(resetControllerString,      asynParamInt32, &this->resetController_);

  //Create Axis parameters
  createParam(axisStatusResetString,      asynParamInt32, &this->axisStatusReset_);
  createParam(axisResetString,            asynParamInt32, &this->axisReset_);
  createParam(axisStatusString,           asynParamInt32, &this->axisStatus_);
  createParam(homingProcedureString,      asynParamInt32, &this->homingProcedure_);
  createParam(axisModeString,             asynParamInt32, &this->axisMode_);
  createParam(mopOffsetPosString,         asynParamInt32, &this->mopOffsetPos_);
  createParam(mopOffsetNegString,         asynParamInt32, &this->mopOffsetNeg_);
  createParam(stepResolutionString,       asynParamInt32, &this->stepResolution_);
  createParam(stopCurrentString,          asynParamInt32, &this->stopCurrent_);
  createParam(runCurrentString,           asynParamInt32, &this->runCurrent_);
  createParam(boostCurrentString,         asynParamInt32, &this->boostCurrent_);
  createParam(encoderTypeString,          asynParamInt32, &this->encoderType_);
  createParam(initRecoveryTimeString,     asynParamInt32, &this->initRecoveryTime_);
  createParam(positionRecoveryTimeString, asynParamInt32, &this->positionRecoveryTime_);
  createParam(boostConditionString,       asynParamInt32, &this->boost_);
  createParam(encoderRateString,          asynParamInt32, &this->encoderRate_);
  createParam(switchTypString,            asynParamInt32, &this->switchTyp_);
  createParam(pwrStageModeString,         asynParamInt32, &this->pwrStageMode_);
  createParam(encoderResolutionString,    asynParamInt32, &this->encoderRes_);
  createParam(encoderFunctionString,      asynParamInt32, &this->encoderFunc_);
  createParam(encoderSFIWidthString,      asynParamInt32, &this->encoderSFIWidth_);
  createParam(encoderDirectionString,     asynParamInt32, &this->encoderDirection_);
  createParam(powerStagetMonitorString,   asynParamInt32, &this->powerStageMonitor_);
  createParam(currentDelayTimeString,     asynParamInt32, &this->currentDelayTime_);
  createParam(powerStageTempString,       asynParamFloat64, &this->powerStageTemp_);
  createParam(motorTempString,            asynParamFloat64, &this->motorTemp_);
  createParam(axisBrakeOutputString,      asynParamFloat64, &this->axisBrakeOutput_);
  createParam(axisDisableMotorString,     asynParamInt32, &this->axisDisableMotor_);
  createParam(axisBrakeEngageTimeString,  asynParamFloat64, &this->axisBrakeEngageTime_);
  createParam(axisBrakeReleaseTimeString, asynParamFloat64, &this->axisBrakeReleaseTime_);

  /* Connect to phytron controller */
  status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
      "%s: cannot connect to phytron controller\n",
      functionName);
  } else {
    // this hook forces an initial readout before the motor record has
    // finished initializing (before pass 1), so it has actual values for
    // REP, RMP, MSTA fields, especially for absolute encoders
    if (controllers.empty())
      initHookRegister(&phytronController::epicsInithookFunction);

    //phytronCreateAxis will search for the controller for axis registration
    controllers.push_back(this);

    //RESET THE CONTROLLER
    //if (sendPhytronCommand(std::string("CR")))
    //  asynPrint(this->pasynUserSelf, ASYN_TRACE_WARNING,
    //        "phytronController::phytronController: Could not reset controller %s\n", this->controllerName_);

    //Wait for reset to finish
    //epicsThreadSleep(10.0);

    startPoller(movingPollPeriod, idlePollPeriod, 5);
  }
}

/** Creates a new phytronController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] phytronPortName   The name of the drvAsynIPPPort that was created previously to connect to the phytron controller
  * \param[in] numController     number of axes that this controller supports is numController*AXES_PER_CONTROLLER
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int phytronCreateController(const char *phytronPortName, const char *asynPortName,
                                   int movingPollPeriod, int idlePollPeriod, double timeout)
{
  new phytronController(phytronPortName, asynPortName, movingPollPeriod/1000., idlePollPeriod/1000., timeout);
  return asynSuccess;
}

/** asynUsers use this to read integer parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[out] value      Parameter value
 */
asynStatus phytronController::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
  phytronAxis   *pAxis;
  phytronStatus phyStatus;
  std::string   sResponse;
  int           iParameter(0);

  //Call base implementation first
  asynMotorController::readInt32(pasynUser, value);

  //Check if this is a call to read a controller parameter
  if(pasynUser->reason == resetController_ || pasynUser->reason == controllerStatusReset_){
    //Called only on initialization of bo records RESET and RESET-STATUS
    return asynSuccess;
  } else if (pasynUser->reason == controllerStatus_){
    phyStatus = sendPhytronCommand(std::string("ST"), sResponse);
    CHECK_CTRL("readInt32", "Reading controller status", return phyToAsyn(phyStatus));

    *value = atoi(sResponse.c_str());
    return asynSuccess;
  }

  //This is an axis request, find the axis
  pAxis = getAxis(pasynUser);
  if(!pAxis){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
       "phytronController::readInt32: Axis not found on the controller %s\n", this->controllerName_);
    return asynError;
  }

  if(pasynUser->reason == homingProcedure_){
    getIntegerParam(pAxis->axisNo_, homingProcedure_, value);
    return asynSuccess;
  } else if (pasynUser->reason == axisReset_ || pasynUser->reason == axisStatusReset_){
    //Called only on initialization of AXIS-RESET and AXIS-STATUS-RESET bo records
    return asynSuccess;
  } else if (pasynUser->reason == axisDisableMotor_) {
    *value = pAxis->disableMotor_ & 1;
    return asynSuccess;
  }
  else if (pasynUser->reason == axisMode_)             iParameter =  1;
  else if (pasynUser->reason == mopOffsetPos_)         iParameter = 11;
  else if (pasynUser->reason == mopOffsetNeg_)         iParameter = 12;
  else if (pasynUser->reason == stepResolution_)       iParameter = 45;
  else if (pasynUser->reason == stopCurrent_)          iParameter = 40;
  else if (pasynUser->reason == runCurrent_)           iParameter = 41;
  else if (pasynUser->reason == boostCurrent_)         iParameter = 42;
  else if (pasynUser->reason == encoderType_)          iParameter = 43;
  else if (pasynUser->reason == initRecoveryTime_)     iParameter = 13;
  else if (pasynUser->reason == positionRecoveryTime_) iParameter = 16;
  else if (pasynUser->reason == boost_)                iParameter = 17;
  else if (pasynUser->reason == encoderRate_)          iParameter = 26;
  else if (pasynUser->reason == switchTyp_)            iParameter = 27;
  else if (pasynUser->reason == pwrStageMode_)         iParameter = 28;
  else if (pasynUser->reason == encoderRes_)           iParameter = 35;
  else if (pasynUser->reason == encoderFunc_)          iParameter = 36;
  else if (pasynUser->reason == encoderSFIWidth_)      iParameter = 37;
  else if (pasynUser->reason == encoderDirection_)     iParameter = 38;
  else if (pasynUser->reason == currentDelayTime_)     iParameter = 43;
  else if (pasynUser->reason == powerStageMonitor_)    iParameter = 53;
  else return asynSuccess; // ignore unhandled request

  phyStatus = sendPhytronCommand(std::string("M") + pAxis->axisModuleNo_ + "P" + std::to_string(iParameter) + "R",
                                 sResponse);
  CHECK_AXIS("readInt32", "Reading parameter", pAxis, return phyToAsyn(phyStatus));

  *value = atoi(sResponse.c_str());

  //{STOP,RUN,BOOST} current records have EGU set to mA, but device returns 10mA
  if(pasynUser->reason == stopCurrent_ || pasynUser->reason == runCurrent_ ||
      pasynUser->reason == boostCurrent_)
    *value *= 10;

  return asynSuccess;
}

/** asynUsers use this to write integer parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[in] value       Parameter value to be written
 */
asynStatus phytronController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  phytronAxis   *pAxis;
  phytronStatus phyStatus(phytronSuccess);

  //Call base implementation first
  asynMotorController::writeInt32(pasynUser, value);

  /*
   * Check if this is a call to reset the controller, else it is an axis request
   */
  if(pasynUser->reason == resetController_){
    phyStatus = sendPhytronCommand(std::string("CR"));
    CHECK_CTRL("writeInt32", "Reseting controller", );
    resetAxisEncoderRatio();
    return phyToAsyn(phyStatus);
  } else if(pasynUser->reason == controllerStatusReset_){
    phyStatus = sendPhytronCommand(std::string("STC"));
    CHECK_CTRL("writeInt32", "Reseting status", );
    return phyToAsyn(phyStatus);
  }
  /*
   * This is an axis request, find the axis
   */
  pAxis = getAxis(pasynUser);
  if(!pAxis){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
       "phytronAxis::writeInt32: Axis not found on the controller %s\n", this->controllerName_);
    return asynError;
  }

  this->outString_[0] = '\0';
  if(pasynUser->reason == homingProcedure_){
    setIntegerParam(pAxis->axisNo_, pasynUser->reason, value);
    callParamCallbacks();
    return asynSuccess;
  } else if(pasynUser->reason == axisReset_){
    sprintf(this->outString_, "M%sC", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == axisStatusReset_){
    sprintf(this->outString_, "SEC%s", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == axisMode_){
    sprintf(this->outString_, "M%sP01=%d", pAxis->axisModuleNo_,value);
  } else if(pasynUser->reason == mopOffsetPos_){
    sprintf(this->outString_, "M%sP11=%d", pAxis->axisModuleNo_,value);
  } else if(pasynUser->reason == mopOffsetNeg_){
    sprintf(this->outString_, "M%sP12=%d", pAxis->axisModuleNo_,value);
  } else if (pasynUser->reason == stepResolution_){
    sprintf(this->outString_, "M%sP45=%d", pAxis->axisModuleNo_,value);
  }  else if (pasynUser->reason == stopCurrent_){
    value /= 10; //STOP_CURRENT record has EGU mA, device expects 10mA
    sprintf(this->outString_, "M%sP40=%d", pAxis->axisModuleNo_,value);
  } else if (pasynUser->reason == runCurrent_){
    value /= 10; //RUN_CURRENT record has EGU mA, device expects 10mA
    sprintf(this->outString_, "M%sP41=%d", pAxis->axisModuleNo_,value);
  } else if (pasynUser->reason == boostCurrent_){
    value /= 10; //BOOST_CURRENT record has EGU mA, device expects 10mA
    sprintf(this->outString_, "M%sP42=%d", pAxis->axisModuleNo_,value);
  } else if (pasynUser->reason == encoderType_){
    sprintf(this->outString_, "M%sP34=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == initRecoveryTime_){
    sprintf(this->outString_, "M%sP13=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == positionRecoveryTime_){
    sprintf(this->outString_, "M%sP16=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == boost_){
    sprintf(this->outString_, "M%sP17=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == encoderRate_){
    sprintf(this->outString_, "M%sP26=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == switchTyp_){
    sprintf(this->outString_, "M%sP27=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == pwrStageMode_){
    sprintf(this->outString_, "M%sP28=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == encoderRes_){
    sprintf(this->outString_, "M%sP35=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == encoderFunc_){
    //Value is VAL field of parameter P37 record. If P37 is positive P36 is set to 1, else 0
    sprintf(this->outString_, "M%sP36=%d", pAxis->axisModuleNo_, value > 0 ? 1 : 0);
  } else if(pasynUser->reason == encoderSFIWidth_){
    sprintf(this->outString_, "M%sP37=%d", pAxis->axisModuleNo_, value);
  } else if(pasynUser->reason == encoderSFIWidth_){
    sprintf(this->outString_, "M%sP38=%d", pAxis->axisModuleNo_, value);
  } else if(pasynUser->reason == powerStageMonitor_){
    sprintf(this->outString_, "M%sP53=%d", pAxis->axisModuleNo_, value);
  } else if(pasynUser->reason == currentDelayTime_){
    sprintf(this->outString_, "M%sP43=%d", pAxis->axisModuleNo_, value);
  } else if(pasynUser->reason == encoderDirection_){
    sprintf(this->outString_, "M%sP38=%d", pAxis->axisModuleNo_, value);
  } else if(pasynUser->reason == axisDisableMotor_){
    pAxis->disableMotor_ = (pAxis->disableMotor_ & (~1)) | (value != 0) ? 1 : 0;
    pAxis->setBrakeOutput(NULL, pAxis->brakeReleased_);
    return asynSuccess;
  }

  if (this->outString_[0]) {
    phyStatus = sendPhytronCommand(const_cast<const char*>(this->outString_));
    CHECK_AXIS("writeInt32", "Writing parameter", pAxis, return phyToAsyn(phyStatus));
  }

  return asynSuccess;
}

/** asynUsers use this to read float parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[out] value      Parameter value
 */
asynStatus phytronController::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
  phytronAxis   *pAxis;
  phytronStatus phyStatus(phytronSuccess);
  std::string   sResponse;

  pAxis = getAxis(pasynUser);
  if(!pAxis){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
       "phytronController::readFloat64: Axis not found on the controller %s\n", this->controllerName_);
    return asynError;
  }

  //Call base implementation first
  asynMotorController::readFloat64(pasynUser, value);

  this->outString_[0] = '\0';
  if(pasynUser->reason == powerStageTemp_){
    sprintf(this->outString_, "M%sP49R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == motorTemp_){
    sprintf(this->outString_, "M%sP54R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == axisBrakeOutput_){
    *value = pAxis->brakeOutput_;
    return asynSuccess;
  } else if(pasynUser->reason == axisBrakeEngageTime_){
    *value = pAxis->brakeEngageTime_;
    return asynSuccess;
  } else if(pasynUser->reason == axisBrakeReleaseTime_){
    *value = pAxis->brakeReleaseTime_;
    return asynSuccess;
  }

  if (this->outString_[0]){
    phyStatus = sendPhytronCommand(const_cast<const char*>(this->outString_), sResponse);
    CHECK_AXIS("readFloat64", "Reading parameter", pAxis, return phyToAsyn(phyStatus));

    //Power stage and motor temperature records have EGU °C, but device returns 0.1 °C
    *value = atof(sResponse.c_str()) / 10.;
  }

  return asynSuccess;
}

/** asynUsers use this to write float parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[in] value       Parameter value to be written
 */
asynStatus phytronController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  phytronAxis* pAxis(getAxis(pasynUser));
  if(!pAxis){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
       "phytronAxis::writeFloat64: Axis not found on the controller %s\n", this->controllerName_);
    return asynError;
  }

  if (pasynUser->reason == axisBrakeOutput_)
    pAxis->brakeOutput_ = floor(10.0 * ((value > -100.0 && value < 100.) ? value : 0.0) + 0.5) / 10.0;
  else if (pasynUser->reason == axisBrakeEngageTime_) {
    if (value > MAXIMUM_BRAKE_TIME)
      value = MAXIMUM_BRAKE_TIME;
    pAxis->brakeEngageTime_ = value > 0.0 ? value : 0.0;
  } else if (pasynUser->reason == axisBrakeReleaseTime_) {
    if (value > MAXIMUM_BRAKE_TIME)
      value = MAXIMUM_BRAKE_TIME;
    pAxis->brakeReleaseTime_ = value > 0.0 ? value : 0.0;
  }

  //Call base implementation
  return asynMotorController::writeFloat64(pasynUser, value);
}

/** Called when asyn clients call pasynOption->read().
 * The base class implementation simply prints an error message.
 * Derived classes may reimplement this function if required.
 * \param[in]  pasynUser pasynUser structure that encodes the reason and address.
 * \param[in]  key       Option key string.
 * \param[out] value     string to be returned
 * \param[in]  maxChars  Size of value string
 * \return asyn status
 */
asynStatus phytronController::readOption(asynUser *pasynUser, const char *key, char *value, int maxChars)
{
  if (key && value) {
    *value = '\0';
    if (epicsStrCaseCmp(key, "pollMethod") == 0) {
      const char* szMethod;
      phytronAxis* pAxis(getAxis(pasynUser));
      enum pollMethod iMethod(pAxis ? pAxis->iPollMethod_ : iDefaultPollMethod_);
      switch (iMethod) {
        case pollMethodDefault:            szMethod = "default";       break;
        case pollMethodSerial:             szMethod = "serial";        break;
        case pollMethodAxisParallel:       szMethod = "axis-parallel"; break;
        case pollMethodControllerParallel: szMethod = "ctrl-parallel"; break;
        default:                           szMethod = "???";           break;
      }
      snprintf(value, maxChars, "%d/%s", static_cast<int>(iMethod), szMethod);
      return asynSuccess;
    }
  }
  return asynMotorController::readOption(pasynUser, key, value, maxChars);
}

/** Called when asyn clients call pasynOption->write().
 * The base class implementation simply prints an error message.
 * Derived classes may reimplement this function if required.
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] key Option key string.
 * \param[in] value Value string.
 * \return asyn status
 */
asynStatus phytronController::writeOption(asynUser *pasynUser, const char *key, const char *value)
{
  phytronAxis* pAxis(NULL);
  enum pollMethod iMethod(pollMethodSerial);
  epicsInt64 iTmp(-2);
  int iAxisNo(0);
  size_t iLen;

  if (!key || !value)
    goto finish;
  if (epicsStrCaseCmp(key, "pollMethod") != 0)
    goto finish;

  getAddress(pasynUser, &iAxisNo);
  if (iAxisNo > 0) {
    pAxis = getAxis(iAxisNo);
    if (!pAxis) {
      epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                    "phytronController:writeOption(%s, %s) wrong axis", key, value);
      return asynError;
    }
    iMethod = pollMethodDefault;
  }
  if (epicsParseInt64(value, &iTmp, 0, NULL) == 0) {
    if (iTmp < (pAxis ? static_cast<epicsInt64>(pollMethodDefault) : static_cast<epicsInt64>(pollMethodSerial)) ||
        iTmp > static_cast<epicsInt64>(pollMethodControllerParallel))
      goto wrongValue;
    iMethod = static_cast<pollMethod>(iTmp);
  } else {
    while (isspace(*value)) ++value;
    iLen = strlen(value);
    while (iLen > 0 && isspace(value[iLen - 1])) --iLen;

    if (pAxis && ((iLen ==  7 && !epicsStrnCaseCmp(value, "default", 7)) ||
                  (iLen ==  8 && !epicsStrnCaseCmp(value, "standard", 8)) ||
                  (iLen == 10 && !epicsStrnCaseCmp(value, "controller", 10))))
      iMethod = pollMethodDefault;
    else if ((iLen ==  6 && !epicsStrnCaseCmp(value, "serial", 6)) ||
             (iLen == 11 && !epicsStrnCaseCmp(value, "no-parallel", 11)) ||
             (iLen == 12 && !epicsStrnCaseCmp(value, "not-parallel", 12)) ||
             (iLen ==  3 && !epicsStrnCaseCmp(value, "old", 3)))
      iMethod = pollMethodSerial;
    else if ((iLen ==  4 && !epicsStrnCaseCmp(value, "axis", 4)) ||
             (iLen == 13 && !epicsStrnCaseCmp(value, "axis-parallel", 13)))
      iMethod = pollMethodAxisParallel;
    else if ((iLen ==  4 && !epicsStrnCaseCmp(value, "ctrl", 4)) ||
             (iLen ==  8 && !epicsStrnCaseCmp(value, "parallel", 8)) ||
             (iLen == 13 && !epicsStrnCaseCmp(value, "ctrl-parallel", 13)) ||
             (iLen == 19 && !epicsStrnCaseCmp(value, "controller-parallel", 19)))
      iMethod = pollMethodControllerParallel;
    else {
wrongValue:
      epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                    "phytronController:writeOption(%s, %s) wrong value", key, value);
      return asynError;
    }
  }
  if (pAxis)
    pAxis->iPollMethod_ = iMethod;
  else
    iDefaultPollMethod_ = iMethod;
  return asynSuccess;

finish:
  return asynMotorController::writeOption(pasynUser, key, value);
}

/*
 * Reset the motorEncoderRatio to 1 after the reset of MCM unit
 */
void phytronController::resetAxisEncoderRatio()
{
  for(uint32_t i = 0; i < axes.size(); i++){
    setDoubleParam(axes[i]->axisNo_, motorEncoderRatio_, 1);
  }
}


/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void phytronController::report(FILE *fp, int level)
{
  fprintf(fp, "Phytron phyMOTION motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an phytronAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number.
  */
phytronAxis* phytronController::getAxis(asynUser *pasynUser)
{
  return dynamic_cast<phytronAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an phytronAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number.
  */
phytronAxis* phytronController::getAxis(int axisNo)
{
  return dynamic_cast<phytronAxis*>(asynMotorController::getAxis(axisNo));
}

/** Polls the controller depending on configuration (asynSetOption "pollMethod").
 * This implementation tracks, if it was called at least once.
 * The Option "pollMethod" configures, how to talk to the controller:
 * * pollMethodSerial (old and default):
 *   - 4 single command request and replies: motor position, encoder position,
 *     moving status, axis status (limit switches, home switches, ...)
 *   - takes about 40ms per axis
 * * pollMethodAxisParallel or
 * * pollMethodControllerParallel:
 *   - send a longer command string with multiple commands to the controller
 *   - parses the replies of it (count should be equal to command count)
 *   - takes about 10ms per command string, which saves time
 *   - per axis commands are only axis status, motor position, encoder position
 *     the moving status is part axis status (bit 0)
 *   - pollMethodAxisParallel combines commands for one axis only
 *   - pollMethodControllerParallel tries to combine commands to all axes
 * \return status of the base class
 */
asynStatus phytronController::poll()
{
  asynStatus iResult;
  do_initial_readout_ = false;
  iResult = asynMotorController::poll();
  if (iResult == asynSuccess) {
    // check, which axes should be handled here
    std::vector<phytronAxis*> apTodoAxis;
    for (std::vector<phytronAxis*>::iterator it = axes.begin(); it != axes.end(); ++it) {
      phytronAxis* pAxis(*it);
      if (!pAxis) continue;
      pollMethod iPollMethod(pAxis->iPollMethod_);
      if (iPollMethod < 0) iPollMethod = iDefaultPollMethod_;
      // this axis is configured for controller parallel poll and handled here
      if (iPollMethod == pollMethodControllerParallel)
        apTodoAxis.emplace_back(std::move(pAxis));
    }
    if (!apTodoAxis.empty()) {
      std::vector<std::string> asRequests, asAnswers;
      asRequests.reserve(3 * apTodoAxis.size());
      // collect requests
      for (std::vector<phytronAxis*>::iterator it = apTodoAxis.begin(); it != apTodoAxis.end(); ++it)
        (*it)->appendRequestString(asRequests);
      // communicate
      iResult = phyToAsyn(sendPhytronMultiCommand(asRequests, asAnswers, true));
      if (iResult == asynSuccess) {
        // handle answers
        for (std::vector<phytronAxis*>::iterator it = apTodoAxis.begin(); it != apTodoAxis.end(); ++it)
          if (!(*it)->parseAnswer(asAnswers))
            iResult = asynError;
        if (!asAnswers.empty())
          iResult = asynError;
      }
    }
  }
  return iResult;
}

/**
 * This hook function makes sure, that the IOC initialization waits until all
 * controllers were read at least once. This allows the motor record to use
 * initialized values (at pass 1), especially if an encoder is available and
 * was configured to be used. Every controller has its own poller thread, this
 * function is called by the main thread, so this blocking is not a dead lock.
 * \param[in] iState new state information
 */
void phytronController::epicsInithookFunction(initHookState iState)
{
  if (iState != initHookAfterInitDevSup)
    return;

  // before motorRecord init_record pass 1
  for (vector<phytronController*>::iterator itC = controllers.begin();
       itC != controllers.end(); ++itC) {
    // iterate over all controllers
    phytronController* pC(*itC);
    while (pC) {
      // wait until the controller was polled once
      bool did_initial_readout(false);
      pC->lock();
      did_initial_readout = !pC->do_initial_readout_;
      pC->unlock();
      if (did_initial_readout) // done
        break;
      epicsThreadSleep(0.1); // not done: wait some time
    }
  }
}

/**
 * \brief implements phytron specific data format:
 *        send commands to Phytron controller as one command line, await
 *        answer and check for ACK/NAK.
 *        Note: Phytron told us, that the controller handles up to 1000 bytes of input and 2000 bytes on output.
 * \param[in]  sCommand   input command line
 * \param[out] sResponse  answer output
 * \param[in]  bACKonly   false: allow ACK and NAK, true: allow ACK only
 * \return communication status
 */
phytronStatus phytronController::sendPhytronCommand(std::string sCommand, std::string &sResponse, bool bACKonly)
{
  if (sCommand.size() > 994) // maximum command length exceeded (1000 bytes incl. framing)
    return phytronInvalidCommand;
  if (sCommand.capacity() < sCommand.size() + 6)
    sCommand.reserve(sCommand.size() + 6);
  sCommand.insert(0, "0"); // address of controller, TODO: allow different values for serial connection
  sCommand.push_back(':'); // separator for CRC
  epicsUInt8 byCRC(0);
  for (auto it = sCommand.begin(); it != sCommand.end(); ++it)
    byCRC ^= static_cast<epicsUInt8>(*it); // "calculate" CRC
  epicsUInt8 byCRC2(byCRC & 0x0F);
  byCRC >>= 4;
  // put framing incl. CRC
  sCommand.insert(0, "\x02"); // STX
  sCommand.push_back(static_cast<char>(byCRC  + ((byCRC  > 9) ? ('A' - 10) : '0')));
  sCommand.push_back(static_cast<char>(byCRC2 + ((byCRC2 > 9) ? ('A' - 10) : '0')));
  sCommand.push_back('\x03'); // ETX

  // communicate
  sResponse.resize(2010); // this is the maximum expected answer size: 2000 bytes plus framing
  size_t nRead(0);
  phytronStatus iResult(static_cast<phytronStatus>(writeReadController(sCommand.c_str(), &sResponse[0], sResponse.size(), &nRead, timeout_)));
  if (iResult != phytronSuccess)
    return iResult;
  if (nRead < 0 || nRead > sResponse.size())
    return phytronInvalidReturn;
  sResponse.resize(nRead);

  // check answer
  auto it(sResponse.find_first_of('\x02'));
  if (it == std::string::npos)
    return phytronInvalidReturn;
  sResponse.erase(0, ++it); // remove STX and garbage before
  it = sResponse.find_first_of('\x03');
  if (it == std::string::npos)
    return phytronInvalidReturn;
  sResponse.erase(it); // remove ETX and garbage behind
  size_t i(sResponse.size());
  if (i > 3 && sResponse[i - 3] == ':') { // CRC appended
    if (!isalnum(sResponse[i - 2]) || !isalnum(sResponse[i - 1]))
      return phytronInvalidReturn;
    if (toupper(sResponse[i - 2]) != 'X' || toupper(sResponse[i - 1]) != 'X') {
      // check CRC
      byCRC = 0;
      for (size_t j = 0; j < (i - 2); ++j)
        byCRC ^= sResponse[j];
      byCRC2 = byCRC & 0x0F;
      byCRC >>= 4;
      if (toupper(sResponse[i - 2]) != static_cast<char>(byCRC  + ((byCRC  > 9) ? ('A' - 10) : '0')) ||
          toupper(sResponse[i - 1]) != static_cast<char>(byCRC2 + ((byCRC2 > 9) ? ('A' - 10) : '0')))
        return phytronInvalidReturn; // CRC invalid
    }
    sResponse.resize(i - 3); // remove separator and CRC
  }
  if (!sResponse.empty()) {
    switch (sResponse.front()) { // answer should start with ACK or NAK
      case '\x06': // ACK
        if (bACKonly)
          sResponse.erase(0, 1);
        return phytronSuccess;
      case '\x15': // NAK
        if (bACKonly)
          return phytronSuccess;
        break;
      default:
        break;
    }
  }
  return phytronInvalidReturn;
}

/**
 * \brief implements phytron specific data format:
 *        send commands to Phytron controller as one command line, await
 *        answer and check for ACK/NAK.
 *        Note: Phytron told us, that the controller handles up to 1000 bytes of input and 2000 bytes on output.
 * \param[in]  sCommand   input command line
 * \param[out] psResponse pointer to answer output or NULL/nullptr (discard answer)
 * \param[in]  bACKonly   false: allow ACK and NAK, true: allow ACK only
 * \return communication status
 */
phytronStatus phytronController::sendPhytronCommand(std::string sCommand, std::string* psResponse, bool bACKonly)
{
  std::string sTmp;
  if (!psResponse) psResponse = &sTmp;
  return sendPhytronCommand(sCommand, *psResponse, bACKonly);
}

/**
 * \brief Implements phytron specific data format:
 *        send commands to Phytron controller as one command line, await
 *        answer, split answer into separate strings and check for ACK/NAK.
 *        This function will make sure, that a maximum of 42 commands are sent at once.
 *        If there are more, a second (third...) call will be made.
 * \param[in]  asCommands    input vector of command strings
 * \param[out] asResponses   output vector of answers
 * \param[in]  bAllowNAK     false: do not allow NAK, remove ACK characters from answer --> look at return value;
 *                           true: allow NAK, keep ACK/NAK in answers
 * \param[in]  bForceSingle  false: try to combine all requests together
 *                           true: call one by one
 * \return communication status
 */
phytronStatus phytronController::sendPhytronMultiCommand(std::vector<std::string> asCommands,
    std::vector<std::string> &asResponses, bool bAllowNAK, bool bForceSingle)
{
  std::string sCommand, sResponse;
  size_t iCommands(asCommands.size()), iMaxLen(0);
  int iCount, iMaxCmdCount;
  asResponses.clear();

  for (;;) {
    // get needed string length
    iMaxLen = 0;
    iCount = 0;
    iMaxCmdCount = bForceSingle ? 1 : 42;
    for (auto it = asCommands.begin();
         it != asCommands.end() && iCount < iMaxCmdCount; ++it, ++iCount) {
      iMaxLen += it->size() + 1;
      if (it->empty())
        return phytronInvalidCommand;
      if (iMaxLen > 994) // max length for a single command line
        break;
    }
    if (iMaxLen < 1)
      return phytronInvalidCommand;

    // merge single commands to one command line
    sCommand.clear();
    sCommand.reserve(iMaxLen);
    iMaxCmdCount = iCount;
    iCount = 0;
    for (auto it = asCommands.begin();
         it != asCommands.end() && sCommand.size() < iMaxLen && iCount < iMaxCmdCount;
         ++it, ++iCount) {
      if (!sCommand.empty())
        sCommand.append(" ");
      sCommand.append(*it);
    }
    asCommands.erase(asCommands.begin(), asCommands.begin() + iCount);

    // communicate
    phytronStatus iResult(sendPhytronCommand(sCommand, sResponse, false));
    if (iResult != phytronSuccess) return iResult;
    // split answers
    while (!sResponse.empty()) {
      std::string::size_type i;
      for (i = 1; i < sResponse.size(); ++i)
        if (sResponse[i] == '\x06' || sResponse[i] == '\x15')
          break;
      if (!bAllowNAK) {
        if (sResponse[0] != '\x06') // only ACK allowed
          return phytronInvalidReturn;
        asResponses.emplace_back(sResponse.substr(1, i - 1));
      }
      else
        asResponses.emplace_back(sResponse.substr(0, i));
      sResponse.erase(0, i);
    }
    if (asCommands.empty())
      break;
  }
  // number of commands and answers have to match
  return (iCommands == asResponses.size()) ? phytronSuccess : phytronInvalidReturn;
}

/** Castst phytronStatus to asynStatus enumeration
 * \param[in] phyStatus
 */
asynStatus phytronController::phyToAsyn(phytronStatus phyStatus)
{
  if(phyStatus == phytronInvalidReturn || phyStatus == phytronInvalidCommand) return asynError;
  return (asynStatus) phyStatus;
}


//******************************************************************************
//                   PHYTRON AXIS IMPLEMENTATION
//******************************************************************************

/** Creates a new phytronAxis object.
  * Configuration command, called directly or from iocsh
  * \param[in] controllerName    Name of the asyn port created by calling phytronCreateController from st.cmd
  * \param[in] module            Index of the I1AM01 module controlling this axis
  * \param[in] axis              Axis index
  */
extern "C" int phytronCreateAxis(const char* controllerName, int module, int axis)
{
  phytronAxis *pAxis;

  //Find the controller
  uint32_t i;
  for(i = 0; i < controllers.size(); i++){
    if(!strcmp(controllers[i]->controllerName_, controllerName)) {
      pAxis = new phytronAxis(controllers[i], module*10 + axis);
      controllers[i]->axes.push_back(pAxis);
      break;
    }
  }

  //If controller is not found, report error
  if(i == controllers.size()){
    printf("ERROR: phytronCreateAxis: Controller %s is not registered\n", controllerName);
    return asynError;
  }

  return asynSuccess;
}

/** Configures a phytronAxis object to use a brake
 * \param[in] szControllerName    Name of the asyn port created by calling phytronCreateController from st.cmd
 * \param[in] fAxis               axis index (module.index)
 * \param[in] fOutput             digital out index (module.index) or 0 to disable this function, negative value inverts output
 * \param[in] bDisableMotor       0=keep motor enabled, 1=disable idle motor/enable when moved
 * \param[in] dEngageTime         time to engage brake (disable motor after this time in milliseconds, max. 10 sec)
 * \param[in] dReleaseTime        time to release brake (start move after this time in milliseconds, max. 10 sec)
 */
extern "C" int phytronBrakeOutput(const char* szControllerName, float fAxis, float fOutput, int bDisableMotor, double dEngageTime, double dReleaseTime)
{
  char szAxisNo[5];
  uint32_t i, j;
  epicsSnprintf(szAxisNo, sizeof(szAxisNo), "%.1f", fAxis);
  szAxisNo[sizeof(szAxisNo) - 1] = '\0';
  if (!szControllerName || !*szControllerName)
    goto failed;
  for (i = 0; i < controllers.size(); ++i) {
    phytronController* pC(controllers[i]);
    if (!pC || strcmp(pC->controllerName_, szControllerName) != 0)
      continue;
    // found controller, search axis
    for (j = 0; j < pC->axes.size(); ++j) {
      phytronAxis* pA(pC->axes[j]);
      if (!pA || strcmp(pA->axisModuleNo_, szAxisNo) != 0)
        continue;

      // found axis on controller: store data
      return pA->configureBrake(fOutput, bDisableMotor != 0,
                                dEngageTime / 1000., dReleaseTime / 1000.);
    }
    break; // controller found, but not axis
  }

failed:
  printf("ERROR: phytronBrakeOutput: Controller %s is not registered or axis %s not found\n", szControllerName, szAxisNo);
  return asynError;
}

/** Creates a new phytronAxis object.
  * \param[in] pC Pointer to the phytronController to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  *
  * Initializes register numbers, etc.
  */
phytronAxis::phytronAxis(phytronController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo)
  , brakeOutput_(0.0)
  , disableMotor_(0)
  , brakeEngageTime_(0.0)
  , brakeReleaseTime_(0.0)
  , pC_(pC)
  , lastStatus(phytronSuccess)
  , brakeReleased_(0)
  , iPollMethod_(pollMethodDefault)
  , homeState_(0)
{
  //Controller always supports encoder. Encoder enable/disable is set through UEIP
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);
  setDoubleParam(pC_->motorEncoderRatio_, 1);
  epicsSnprintf(axisModuleNo_, sizeof(axisModuleNo_), "%.1f", axisNo / 10.);
  axisModuleNo_[sizeof(axisModuleNo_) - 1] = '\0';
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void phytronAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n",
            axisNo_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

/** Sets velocity parameters before the move is executed. Controller produces a
 * trapezoidal speed profile defined by these parmeters.
 * \param[in,out] pCmdList      list of commands to append to or NULL (execute directly)
 * \param[in]     minVelocity   Start velocity
 * \param[in]     maxVelocity   Maximum velocity
 * \param[in]     moveType      Type of movement determines which controller speed parameters are set
 */
phytronStatus phytronAxis::setVelocity(std::vector<std::string>* pCmdList, double minVelocity, double maxVelocity, int moveType)
{
  std::vector<std::string> asRequests;
  enum pollMethod iPollMethod(iPollMethod_);
  if (iPollMethod < 0) iPollMethod = pC_->iDefaultPollMethod_;
  if (!pCmdList) pCmdList = &asRequests;

  maxVelocity = fabs(maxVelocity);
  minVelocity = fabs(minVelocity);

  if(maxVelocity > MAX_VELOCITY){
    maxVelocity = MAX_VELOCITY;
    asynPrint(pasynUser_, ASYN_TRACE_WARNING,
              "phytronAxis::setVelocity: Failed for axis %d - Velocity %f is to high, setting to"
              "maximum velocity: %d!\n", axisNo_, maxVelocity, MAX_VELOCITY);
  } else if (maxVelocity < MIN_VELOCITY){
    maxVelocity = MIN_VELOCITY;
    asynPrint(pasynUser_, ASYN_TRACE_WARNING,
              "phytronAxis::setVelocity: Failed for axis %d - Velocity %f is to low, setting to"
              "minimum velocity: %d!\n", axisNo_, maxVelocity, MIN_VELOCITY);
  }

  if(minVelocity > MAX_VELOCITY){
    minVelocity = MAX_VELOCITY;
    asynPrint(pasynUser_, ASYN_TRACE_WARNING,
              "phytronAxis::setVelocity: Failed for axis %d - Velocity %f is to high, setting to"
              "maximum velocity: %d!\n", axisNo_, maxVelocity, MAX_VELOCITY);
  } else if (minVelocity < MIN_VELOCITY){
    minVelocity = MIN_VELOCITY;
    asynPrint(pasynUser_, ASYN_TRACE_WARNING,
              "phytronAxis::setVelocity: Failed for axis %d - Velocity %f is to low, setting to"
              "minimum velocity: %d!\n", axisNo_, minVelocity, MIN_VELOCITY);
  }

  switch (moveType) {
    case stdMove:  //Set maximum velocity (P14) and minimum velocity (P04)
      pCmdList->push_back(std::string("M") + axisModuleNo_ + "P14=" + std::to_string(maxVelocity));
      pCmdList->push_back(std::string("M") + axisModuleNo_ + "P04=" + std::to_string(minVelocity));
      break;
    case homeMove: //Set maximum velocity (P08) and minimum velocity (P10)
      pCmdList->push_back(std::string("M") + axisModuleNo_ + "P08=" + std::to_string(maxVelocity));
      pCmdList->push_back(std::string("M") + axisModuleNo_ + "P10=" + std::to_string(minVelocity));
      break;
    default:
      return phytronSuccess;
  }
  if (pCmdList != &asRequests) return phytronSuccess;
  return pC_->sendPhytronMultiCommand(asRequests, asRequests, false, iPollMethod == pollMethodSerial);
}

/** Sets acceleration parameters before the move is executed.
 * \param[in,out] pCmdList      list of commands to append to or NULL (execute directly)
 * \param[in]     acceleration  Acceleration to be used in the move
 * \param[in]     moveType      Type of movement determines which controller acceleration parameters is set
 */
phytronStatus phytronAxis::setAcceleration(std::vector<std::string>* pCmdList, double acceleration, int moveType)
{
  std::string sCommand;
  if (acceleration > MAX_ACCELERATION) {
    acceleration = MAX_ACCELERATION;
    asynPrint(pasynUser_, ASYN_TRACE_WARNING,
              "phytronAxis::setAcceleration: Failed for axis %d - Acceleration %f is to high, "
              "setting to maximum acceleration: %d!\n", axisNo_, acceleration, MAX_ACCELERATION);
  } else if (acceleration < MIN_ACCELERATION) {
    acceleration = MIN_ACCELERATION;
    asynPrint(pasynUser_, ASYN_TRACE_WARNING,
              "phytronAxis::setAcceleration: Failed for axis %d - Acceleration %f is to low, "
              "setting to minimum acceleration: %d!\n", axisNo_, acceleration, MIN_ACCELERATION);
  }
  sCommand = std::string("M") + axisModuleNo_ + ((moveType == homeMove) ? "P09=" : "P15=") +
             std::to_string(acceleration);
  if (!pCmdList)
    return pC_->sendPhytronCommand(sCommand);
  pCmdList->push_back(sCommand);
  return phytronSuccess;
}

/** Configure the brake
 * \param[in] fOutput        digital out index (module.index) or 0 to disable this function, negative value inverts output
 * \param[in] bDisableMotor  0=keep motor enabled, 1=disable idle motor/enable when moved
 * \param[in] dEngageTime    time to engage brake (disable motor after this time in seconds, max. 10 sec)
 * \param[in] dReleaseTime   time to release brake (start move after this time in seconds, max. 10 sec)
 */
asynStatus phytronAxis::configureBrake(float fOutput, bool bDisableMotor, double dEngageTime, double dReleaseTime)
{
  brakeOutput_  = floor(10.0 * ((fOutput > -100.0 && fOutput < 100.0) ? fOutput : 0.0) + 0.5) / 10.0;
  disableMotor_ = (disableMotor_ & (~1)) | (bDisableMotor ? 1 : 0);
  if (dEngageTime  > MAXIMUM_BRAKE_TIME)
    dEngageTime  = MAXIMUM_BRAKE_TIME;
  if (dReleaseTime > MAXIMUM_BRAKE_TIME)
    dReleaseTime = MAXIMUM_BRAKE_TIME;
  brakeEngageTime_  = dEngageTime  > 0.0 ? dEngageTime  : 0.0;
  brakeReleaseTime_ = dReleaseTime > 0.0 ? dReleaseTime : 0.0;
  setBrakeOutput(NULL, brakeReleased_);
  //printf("setting brake for axis %s: output=%.15g disable=%d engagetime=%.15g releasetime=%.15g released=%d\n",
  //       axisModuleNo_, brakeOutput_, disableMotor_, brakeEngageTime_, brakeReleaseTime_, brakeReleased_);
  return asynSuccess;
}

/** Engage or release brake and eventually wait for some time
 * \param[in,out] pCmdList          list of commands to append to or NULL (execute directly)
 * \param[in]     bWantToMoveMotor  flag; false=motor-is-stopped, true=motor-should-be-moved
 * \return on success: phytronSuccess, on error a code
 */
phytronStatus phytronAxis::setBrakeOutput(std::vector<std::string>* pCmdList, bool bWantToMoveMotor)
{
  std::vector<std::string> asRequests;
  //printf("setting brake output for axis %s: wantmove=%d output=%.15g disable=%d engagetime=%.15g releasetime=%.15g released=%d\n",
  //       axisModuleNo_, bWantToMoveMotor, brakeOutput_, disableMotor_, brakeEngageTime_, brakeReleaseTime_, brakeReleased_);
  if (!pCmdList) pCmdList = &asRequests;
  brakeReleased_ = bWantToMoveMotor;
  if (bWantToMoveMotor) {
    // activate motor output
    if (!(disableMotor_ & 2)) {
      pCmdList->push_back(std::string("M") + axisModuleNo_ + "MA");
      disableMotor_ |= 2;
    }
    if (fabs(brakeOutput_) > 0.0) {
      // release brake
      sprintf(pC_->outString_, "A%.1f%c", fabs(brakeOutput_), brakeOutput_ > 0.0 ? 'S' : 'R');
      pCmdList->push_back(const_cast<const char*>(pC_->outString_));
    }

    if (pCmdList == &asRequests || brakeReleaseTime_ > 0.0) {
      pC_->sendPhytronMultiCommand(*pCmdList, asRequests, false, false);
      pCmdList->clear();
    }

    if (brakeReleaseTime_ > 0.0)
      epicsThreadSleep(brakeReleaseTime_);
    return phytronSuccess;
  }

  if (fabs(brakeOutput_) > 0.0) {
    // engage brake
    sprintf(pC_->outString_, "A%.1f%c", fabs(brakeOutput_), brakeOutput_ > 0.0 ? 'R' : 'S');
    pCmdList->push_back(const_cast<const char*>(pC_->outString_));
    if (pCmdList == &asRequests || brakeEngageTime_ > 0.0) {
      pC_->sendPhytronMultiCommand(*pCmdList, asRequests, false, false);
      pCmdList->clear();
    }

    if (brakeEngageTime_ > 0.0)
      epicsThreadSleep(brakeEngageTime_);
  }
  if (disableMotor_ & 1) {
    // deactivate motor output
    sprintf(pC_->outString_, "M%sMD", axisModuleNo_);
    pCmdList->push_back(const_cast<const char*>(pC_->outString_));
    disableMotor_ &= ~2;
    if (pCmdList == &asRequests) {
      pC_->sendPhytronMultiCommand(*pCmdList, asRequests, false, false);
      pCmdList->clear();
    }
  }
  return phytronSuccess;
}

/** Execute the move.
 * \param[in] position      Target position (relative or absolute).
 * \param[in] relative      Is the move absolute or relative
 * \param[in] minVelocity   Lowest velocity of the trapezoidal speed profile.
 * \param[in] maxVelocity   Highest velocity of the trapezoidal speed profile
 * \param[in] acceleration  Acceleration to be used
 */
asynStatus phytronAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  std::vector<std::string> asCommands;
  phytronStatus phyStatus;
  enum pollMethod iPollMethod(iPollMethod_);
  if (iPollMethod < 0) iPollMethod = pC_->iDefaultPollMethod_;

  //NOTE: Check if velocity is different, before setting it.
  phyStatus = setVelocity(&asCommands, minVelocity, maxVelocity, stdMove);
  CHECK_AXIS("move", "Setting the velocity", this, return pC_->phyToAsyn(phyStatus));

  //NOTE: Check if velocity is different, before setting it.
  phyStatus = setAcceleration(&asCommands, acceleration, stdMove);
  CHECK_AXIS("move", "Setting the acceleration", this, return pC_->phyToAsyn(phyStatus));

  setBrakeOutput(&asCommands, 1);
  CHECK_AXIS("move", "Setting the brake", this, return pC_->phyToAsyn(phyStatus));

  if (relative) {
    sprintf(pC_->outString_, "M%s%c%d", axisModuleNo_, position>0 ? '+':'-', abs(NINT(position)));
  } else {
    sprintf(pC_->outString_, "M%sA%d", axisModuleNo_, NINT(position));
  }
  asCommands.push_back(const_cast<const char*>(pC_->outString_));

  phyStatus = pC_->sendPhytronMultiCommand(asCommands, asCommands, false, iPollMethod == pollMethodSerial);
  CHECK_AXIS("move", "Start move", this, return pC_->phyToAsyn(phyStatus));

  return asynSuccess;
}

/** Execute the homing procedure
 * \param[in] minVelocity   Lowest velocity of the trapezoidal speed profile.
 * \param[in] maxVelocity   Highest velocity of the trapezoidal speed profile
 * \param[in] acceleration  Acceleration to be used
 * \param[in] forwards      Direction of homing move
 */
asynStatus phytronAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  std::vector<std::string> asCommands;
  phytronStatus phyStatus;
  int homingType;
  enum pollMethod iPollMethod(iPollMethod_);
  if (iPollMethod < 0) iPollMethod = pC_->iDefaultPollMethod_;

  pC_->getIntegerParam(axisNo_, pC_->homingProcedure_, &homingType);
  if (homingType == limit)
    homeState_ = forwards ? 3 : 2;

  phyStatus =  setVelocity(&asCommands, minVelocity, maxVelocity, homeMove);
  CHECK_AXIS("home", "Setting the velocity", this, return pC_->phyToAsyn(phyStatus));

  phyStatus =  setAcceleration(&asCommands, acceleration, homeMove);
  CHECK_AXIS("home", "Setting the acceleration", this, return pC_->phyToAsyn(phyStatus));

  setBrakeOutput(&asCommands, 1);
  CHECK_AXIS("home", "Setting the brake", this, return pC_->phyToAsyn(phyStatus));

  if(forwards){
    if(homingType == limit) sprintf(pC_->outString_, "M%sR+", axisModuleNo_);
    else if(homingType == center) sprintf(pC_->outString_, "M%sR+C", axisModuleNo_);
    else if(homingType == encoder) sprintf(pC_->outString_, "M%sR+I", axisModuleNo_);
    else if(homingType == limitEncoder) sprintf(pC_->outString_, "M%sR+^I", axisModuleNo_);
    else if(homingType == centerEncoder) sprintf(pC_->outString_, "M%sR+C^I", axisModuleNo_);
    //Homing procedures for rotational movements (no hardware limit switches)
    else if(homingType == referenceCenter) sprintf(pC_->outString_, "M%sRC+", axisModuleNo_);
    else if(homingType == referenceCenterEncoder) sprintf(pC_->outString_, "M%sRC+^I", axisModuleNo_);
  } else {
    if(homingType == limit) sprintf(pC_->outString_, "M%sR-", axisModuleNo_);
    else if(homingType == center) sprintf(pC_->outString_, "M%sR-C", axisModuleNo_);
    else if(homingType == encoder) sprintf(pC_->outString_, "M%sR-I", axisModuleNo_);
    else if(homingType == limitEncoder) sprintf(pC_->outString_, "M%sR-^I", axisModuleNo_);
    else if(homingType == centerEncoder) sprintf(pC_->outString_, "M%sR-C^I", axisModuleNo_);
    //Homing procedures for rotational movements (no hardware limit switches)
    else if(homingType == referenceCenter) sprintf(pC_->outString_, "M%sRC-", axisModuleNo_);
    else if(homingType == referenceCenterEncoder) sprintf(pC_->outString_, "M%sRC-^I", axisModuleNo_);
  }
  asCommands.push_back(const_cast<const char*>(pC_->outString_));

  phyStatus = pC_->sendPhytronMultiCommand(asCommands, asCommands, false, iPollMethod == pollMethodSerial);
  CHECK_AXIS("home", "Start homing", this, return pC_->phyToAsyn(phyStatus));

  return asynSuccess;
}

/** Jog the motor. Direction is determined by sign of the maxVelocity profile
 * \param[in] minVelocity   Lowest velocity of the trapezoidal speed profile.
 * \param[in] maxVelocity   Highest velocity of the trapezoidal speed profile
 * \param[in] acceleration  Acceleration to be used
 */
asynStatus phytronAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  std::vector<std::string> asCommands;
  phytronStatus phyStatus;
  enum pollMethod iPollMethod(iPollMethod_);
  if (iPollMethod < 0) iPollMethod = pC_->iDefaultPollMethod_;

  phyStatus = setVelocity(&asCommands, minVelocity, maxVelocity, stdMove);
  CHECK_AXIS("moveVelocity", "Setting the velocity", this, );

  phyStatus = setAcceleration(&asCommands, acceleration, stdMove);
  CHECK_AXIS("moveVelocity", "Setting the acceleration", this, );

  setBrakeOutput(&asCommands, 1);

  asCommands.push_back(std::string("M") + axisModuleNo_ + (maxVelocity < 0. ? "L-" : "L+"));

  phyStatus = pC_->sendPhytronMultiCommand(asCommands, asCommands, false, iPollMethod == pollMethodSerial);
  CHECK_AXIS("moveVelocity", "Start move", this, return pC_->phyToAsyn(phyStatus));

  return asynSuccess;
}

/** Stop the motor
 * \param[in] acceleration  Deceleration to be used
 */
asynStatus phytronAxis::stop(double acceleration)
{
  std::vector<std::string> asCommands;
  phytronStatus phyStatus;
  enum pollMethod iPollMethod(iPollMethod_);
  if (iPollMethod < 0) iPollMethod = pC_->iDefaultPollMethod_;

  phyStatus = setAcceleration(&asCommands, acceleration, stopMove);
  CHECK_AXIS("stop", "Setting the acceleration", this, );

  asCommands.push_back(std::string("M") + axisModuleNo_ + "S");
  phyStatus = pC_->sendPhytronMultiCommand(asCommands, asCommands, false, iPollMethod == pollMethodSerial);
  CHECK_AXIS("stop", "Stop move", this, return pC_->phyToAsyn(phyStatus));

  return asynSuccess;
}

//NOTE: Use this for step-slip check?
asynStatus phytronAxis::setEncoderRatio(double ratio)
{
  phytronStatus phyStatus;
  double encoderPosition;
  std::string sResponse;

  phyStatus = pC_->sendPhytronCommand(std::string("M") + axisModuleNo_ + "P22R", sResponse);
  if (phyStatus == phytronSuccess)
    if (epicsParseDouble(sResponse.c_str(), &encoderPosition, NULL) != 0)
      phyStatus = phytronInvalidReturn;
  CHECK_AXIS("setEncoderRatio", "Reading encoder position", this, return pC_->phyToAsyn(phyStatus));

  phyStatus = pC_->sendPhytronCommand(std::string("M") + axisModuleNo_ + "P39=" + std::to_string(1. / ratio), sResponse);
  CHECK_AXIS("setEncoderRatio", "Setting ratio", this, return pC_->phyToAsyn(phyStatus));

  setDoubleParam(pC_->motorEncoderPosition_, encoderPosition * ratio);
  return asynSuccess;
}

//NOTE: Keep this for step-slip check?
asynStatus phytronAxis::setEncoderPosition(double position)
{
  return asynError;
}

/** Set the new position of the motor on the controller
 * \param[in] position  New absolute motor position
 */
asynStatus phytronAxis::setPosition(double position)
{
  phytronStatus phyStatus;

  sprintf(pC_->outString_, "M%sP20=%f", axisModuleNo_, position);
  phyStatus = pC_->sendPhytronCommand(const_cast<const char*>(pC_->outString_));
  CHECK_AXIS("setPosition", "Setting motor position", this, return pC_->phyToAsyn(phyStatus));

  return asynSuccess;
}

/** Polls the axis depending on configuration (asynSetOption "pollMethod").
 * pollMethodDefault: apply controller setting for this axis (see below)
 * pollMethodSerial (old or default):
 * - 4 single command request and replies: motor position, encoder position,
 *   moving status, axis status (limit switches, home switches, ...)
 * - takes about 40ms per axis
 * pollMethodAxisParallel:
 * pollMethodControllerParallel:
 * - send a longer command string with multiple commands to the controller
 * - parses the replies of it (count should be equal to command count)
 * - takes about 10ms per command string and reply string, which saves time
 * - per axis commands are only axis status, motor position, encoder position
 *   the moving status is part axis status (bit 0)
 * - pollMethodAxisParallel combines commands for one axis only
 * - pollMethodControllerParallel tries to combine commands to all axes (max. 42 commands)
 *   which goes near the maximum controller possibilities (1000 byte in, 2000 byte out)
 * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false).
 * \return status, if this was successful
 */
asynStatus phytronAxis::poll(bool *moving)
{
  int iDone(0);
  enum pollMethod iPollMethod(iPollMethod_);
  std::vector<std::string> asRequests, asAnswers;
  phytronStatus phyStatus(phytronSuccess);
  if (iPollMethod < 0) iPollMethod = pC_->iDefaultPollMethod_;
  switch (iPollMethod) {
    case pollMethodSerial:
    case pollMethodAxisParallel:
      appendRequestString(asRequests); // get commands
      // communicate
      phyStatus = pC_->sendPhytronMultiCommand(asRequests, asAnswers, false, iPollMethod == pollMethodSerial);
      if (phyStatus == phytronSuccess) // on success: parse answers
	if (!parseAnswer(asAnswers) || !asAnswers.empty())
	  phyStatus = phytronInvalidReturn;
      CHECK_AXIS("poll", "Reading axis", this, setIntegerParam(pC_->motorStatusProblem_, 1); \
        callParamCallbacks(); pC_->phyToAsyn(phyStatus));
      break;
    default:
      break;
  }
  pC_->getIntegerParam(axisNo_, pC_->motorStatusDone_, &iDone);
  *moving = !iDone;
  return asynSuccess;
}

/** Prepare to polls the axis.
 * This function generates commands to reads the axis status, motor and encoder position.
 * These commands are appended to the list and send later. The function
 * \ref "phytronAxis::parseAnswer" is called with the results.
 * \param[in,out] asCommands array to append request commands
 */
void phytronAxis::appendRequestString(std::vector<std::string> &asCommands) const
{
  enum pollMethod iPollMethod(iPollMethod_);
  if (iPollMethod < 0) iPollMethod = pC_->iDefaultPollMethod_;
  const char* pszAxis(&axisModuleNo_[0]);
  if (iPollMethod == pollMethodSerial) {
    // old default method, which uses 4 requests
    asCommands.push_back(std::string("M") + pszAxis + "P20R");
    asCommands.push_back(std::string("M") + pszAxis + "P22R");
    asCommands.push_back(std::string("M") + pszAxis + "==H");
    asCommands.push_back(std::string("SE") + pszAxis);
  } else {
    // new method: fix race condition and substitutes "==H" with result of "SE"-bit 0
    asCommands.push_back(std::string("SE") + pszAxis);
    asCommands.push_back(std::string("M") + pszAxis + "P20R");
    asCommands.push_back(std::string("M") + pszAxis + "P22R");
  }
}

/** Handle result of axis poll: parse answers and it calls setIntegerParam() and
 * setDoubleParam() for each item that was polled and then calls callParamCallbacks()
 * at the end.
 * The commands are generated with help of \ref "phytronAxis::appendRequestString".
 * \param[in,out] asValues array to remove requested answer
 * \return true: successfully parsed and removed strings for this axis, false: error
 */
bool phytronAxis::parseAnswer(std::vector<std::string> &asValues)
{
  enum pollMethod iPollMethod(iPollMethod_);
  if (iPollMethod < 0) iPollMethod = pC_->iDefaultPollMethod_;
  int iAxisStatus, iMotIdx, iEncIdx, iStatIdx, iIdleIdx, iHighLimit, iLowLimit;
  size_t iRemoveCount;
  bool bResult(false), bMoving;
  double dRatio;
  epicsInt32 iOldDone(0);
  if (iPollMethod == pollMethodSerial) {
    // old default method, which uses 4 single requests
    iRemoveCount = 4;
    if (asValues.size() != iRemoveCount) goto finish;
    iMotIdx  = 0; // Mm.aP20R
    iEncIdx  = 1; // Mm.aP22R
    iIdleIdx = 2; // Mm.a==H
    iStatIdx = 3; // SEm.a
  } else {
    // new method: fix race condition and substitutes "==H" with result of "SE"-bit 0
    iRemoveCount = 3;
    if (asValues.size() < iRemoveCount) goto finish;
    iStatIdx = 0;  // SEm.a
    iMotIdx  = 1;  // Mm.aP20R
    iEncIdx  = 2;  // Mm.aP22R
    iIdleIdx = -1; // not used
  }
  if (iPollMethod == pollMethodControllerParallel) {
    for (size_t i = 0; i < iRemoveCount; ++i) {
      if (asValues[i].substr(0, 1) != "\x06") // no ACK found here
        goto finish;
      asValues[i].erase(0, 1);
    }
  }

  // current motor position
  setDoubleParam(pC_->motorPosition_, atof(asValues[iMotIdx].c_str()));

  /* Current encoder position: the encoder position returned by the controller
   * is weighted by the controller resolution. To get the absolute encoder
   * position, the received position must be multiplied by the encoder
   * resolution. */
  pC_->getDoubleParam(axisNo_, pC_->motorEncoderRatio_, &dRatio);
  setDoubleParam(pC_->motorEncoderPosition_, atof(asValues[iEncIdx].c_str()) * dRatio);

  // motor status
  iAxisStatus = atoi(asValues[iStatIdx].c_str());

  // idle/moving status
  pC_->getIntegerParam(axisNo_, pC_->motorStatusDone_, &iOldDone);
  if (iIdleIdx >= 0)
    bMoving = (asValues[iIdleIdx].substr(0, 1) != "E");
  else
    bMoving = (iAxisStatus & 1) != 0;
  setIntegerParam(pC_->motorStatusDone_, bMoving ? 0 : 1);
  if (!iOldDone && !bMoving) // axis stopped
    setBrakeOutput(NULL, 0);

  iHighLimit = (iAxisStatus & 0x10) ? 1 : 0;
  iLowLimit  = (iAxisStatus & 0x20) ? 1 : 0;
  if (homeState_ >> 1) {
    // workaround for home on limit switch
    if (homeState_ & 1)
      iHighLimit = 0;
    else
      iLowLimit = 0;
    switch (homeState_ >> 1) {
      case 1: // homing started, wait for moving
        if (bMoving)
          homeState_ += 2;
        break;
      case 2: // wait for move done
      case 3: // tell record about limit reached
      case 4:
        if (!bMoving) {
          homeState_ += 2;
          if (homeState_ & 1)
            iHighLimit = 1;
          else
            iLowLimit = 1;
        }
        if ((homeState_ >> 1) >= 4)
          homeState_ = 0;
        break;
      default:
        homeState_ = 0;
        break;
    }
  }
  setIntegerParam(pC_->motorStatusHighLimit_, iHighLimit);
  setIntegerParam(pC_->motorStatusLowLimit_,  iLowLimit);
  setIntegerParam(pC_->motorStatusAtHome_, (iAxisStatus & 0x40)/0x40);
  setIntegerParam(pC_->motorStatusHomed_,  (iAxisStatus & 0x08)/0x08);
  setIntegerParam(pC_->motorStatusHome_,   (iAxisStatus & 0x08)/0x08);
  setIntegerParam(pC_->motorStatusSlip_,   (iAxisStatus & 0x4000)/0x4000);
  setIntegerParam(pC_->axisStatus_, iAxisStatus); // Update the axis status record ($(P)$(M)_STATUS)
  bResult = true;

finish:
  setIntegerParam(pC_->motorStatusProblem_, bResult ? 0 : 1);
  callParamCallbacks();
  asValues.erase(asValues.begin(), asValues.begin() + iRemoveCount);
  return bResult;
}

/** Parameters for iocsh phytron axis registration*/
static const iocshArg phytronCreateAxisArg0 = {"Controller Name", iocshArgString};
static const iocshArg phytronCreateAxisArg1 = {"Module index", iocshArgInt};
static const iocshArg phytronCreateAxisArg2 = {"Axis index", iocshArgInt};
static const iocshArg* const phytronCreateAxisArgs[] = {&phytronCreateAxisArg0,
                                                      &phytronCreateAxisArg1,
                                                      &phytronCreateAxisArg2};

/** Parameters for iocsh phytron controller registration */
static const iocshArg phytronCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg phytronCreateControllerArg1 = {"PhytronAxis port name", iocshArgString};
static const iocshArg phytronCreateControllerArg2 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg phytronCreateControllerArg3 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg phytronCreateControllerArg4 = {"Timeout (ms)", iocshArgDouble};
static const iocshArg * const phytronCreateControllerArgs[] = {&phytronCreateControllerArg0,
                                                             &phytronCreateControllerArg1,
                                                             &phytronCreateControllerArg2,
                                                             &phytronCreateControllerArg3,
                                                             &phytronCreateControllerArg4};

/** Parameters for iocsh phytron brake(s) output registration */
static const iocshArg phytronBrakeOutputArg0 = {"Controller Name", iocshArgString};
static const iocshArg phytronBrakeOutputArg1 = {"Axis index", iocshArgDouble};
static const iocshArg phytronBrakeOutputArg2 = {"Output index", iocshArgDouble};
static const iocshArg phytronBrakeOutputArg3 = {"Motor disable flag", iocshArgInt};
static const iocshArg phytronBrakeOutputArg4 = {"Brake engage wait time (ms)", iocshArgDouble};
static const iocshArg phytronBrakeOutputArg5 = {"Brake release wait time (ms)", iocshArgDouble};
static const iocshArg* const phytronBrakeOutputArgs[] = {&phytronBrakeOutputArg0,
                                                         &phytronBrakeOutputArg1,
                                                         &phytronBrakeOutputArg2,
                                                         &phytronBrakeOutputArg3,
                                                         &phytronBrakeOutputArg4,
                                                         &phytronBrakeOutputArg5};

static const iocshFuncDef phytronCreateAxisDef = {"phytronCreateAxis", 3, phytronCreateAxisArgs};
static const iocshFuncDef phytronCreateControllerDef = {"phytronCreateController", 5, phytronCreateControllerArgs};
static const iocshFuncDef phytronBrakeOutputDef = {"phytronBrakeOutput", 6, phytronBrakeOutputArgs};

static void phytronCreateControllerCallFunc(const iocshArgBuf *args)
{
  phytronCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].dval);
}

static void phytronCreateAxisCallFunc(const iocshArgBuf *args)
{
  phytronCreateAxis(args[0].sval, args[1].ival, args[2].ival);
}

static void phytronBrakeOutputCallFunc(const iocshArgBuf *args)
{
  phytronBrakeOutput(args[0].sval, args[1].dval, args[2].dval, args[3].ival, args[4].dval, args[5].dval);
}

static void phytronRegister(void)
{
  iocshRegister(&phytronCreateControllerDef, phytronCreateControllerCallFunc);
  iocshRegister(&phytronCreateAxisDef, phytronCreateAxisCallFunc);
  iocshRegister(&phytronBrakeOutputDef, phytronBrakeOutputCallFunc);
}

extern "C" {
epicsExportRegistrar(phytronRegister);
}
