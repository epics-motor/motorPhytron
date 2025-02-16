/*
SPDX-License-Identifier: EPICS
FILENAME... phytronAxisMotor.cpp
USAGE...    Motor driver support for Phytron Axis controller.

Tom Slejko & Bor Marolt
Cosylab d.d. 2014

Lutz Rossa, Helmholtz-Zentrum Berlin fuer Materialien und Energy GmbH, 2021-2025

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
#include <epicsThread.h>
#include <epicsStdlib.h>
#include <epicsString.h>
#include <epicsTime.h>
#include <epicsMath.h>
#include <epicsExit.h>
#include <cantProceed.h>
#include <asynOctetSyncIO.h>

#include "phytronAxisMotor.h"
#include "phytronIOctrl.h"

//******************************************************************************
//                   DEFINES and GLOBAL VARIABLES
//******************************************************************************

using namespace std;

#ifndef ASYN_TRACE_WARNING
#define ASYN_TRACE_WARNING ASYN_TRACE_ERROR
#endif

#define CHECK_CTRL(xfunc, xtxt, xextra) \
    if (phyStatus) { \
      if (phyStatus != lastStatus_) { \
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, \
          "phytronController::%s: (%s) %s failed with error code: %d, reason: %d\n", \
          xfunc, this->controllerName_, xtxt, phyStatus, pasynUser->reason); \
        lastStatus_ = phyStatus; \
      } \
      xextra; \
    } \
    lastStatus_ = phyStatus

#define CHECK_AXIS(xfunc, xtxt, xinstance, xextra) \
    if (phyStatus) { \
      if (phyStatus != lastStatus_) { \
        asynPrint(xinstance->pasynUser_, ASYN_TRACE_ERROR, \
          "phytronAxis::%s: (%s) %s for axis %d failed with error code: %d\n", \
          xfunc, xinstance->pC_->controllerName_, xtxt, xinstance->axisNo_, phyStatus); \
        lastStatus_ = phyStatus; \
      } \
      xextra; \
    } \
    lastStatus_ = phyStatus

//Used for casting position doubles to integers
#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

//Specify maximum sleep time after brake was (de)activated in seconds
#define MAXIMUM_BRAKE_TIME 10.0

//static size of fixed array
#define ARRAY_SIZE(x) ((int)(sizeof(x) / sizeof((x)[0])))

/*
 * Contains phytronController instances, phytronCreateAxis uses it to find and
 * bind axis object to the correct controller object.
 */
std::vector<phytronController*> phytronController::controllers_;

//******************************************************************************
//                   PHYTRON CONTROLLER IMPLEMENTATION
//******************************************************************************

/** Creates a new phytronController object.
 * \param[in] iCtrlType         Controller type: phyMOTION or MCC type
 * \param[in] phytronPortName   The name of the drvAsynIPPort that was created previously to connect to the phytron controller
 * \param[in] asynPortName      The name of the asyn port that will be created for this driver
 * \param[in] address           Serial address of contoller 0..15 (hardware selector)
 * \param[in] movingPollPeriod  The time between polls when any axis is moving
 * \param[in] idlePollPeriod    The time between polls when no axis is moving
 * \param[in] timeout           Communication timeout in ms
 * \param[in] noResetAtBoot     if 0 or not set then controller is reset, if 1 then it's not
 */
phytronController::phytronController(phytronController::TYPE iCtrlType, const char* szPhytronPortName, const char* szAsynPortName,
                                     int iAddress, double iMovingPollPeriod, double iIdlePollPeriod, double dTimeout, bool iNoResetAtBoot)
  :  asynMotorController(szPhytronPortName,
                         0xFF,
                         NUM_PHYTRON_PARAMS,
                         asynOctetMask | asynOptionMask, // additional interfaces
                         asynOctetMask, // additional callback interfaces
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)// Default priority and stack size

  , iCtrlType_(iCtrlType)
  , iAddress_(iAddress)
  , timeout_(dTimeout)
  , lastStatus_(phytronSuccess)
  , do_initial_readout_(true)
  , iDefaultPollMethod_(pollMethodSerial)
  , fake_homed_enable_(false)
  , allow_exit_on_error_(false)
  , statusResetTime_(0.0)
{
  asynStatus status;
  static const char *functionName = "phytronController::phytronController";

  memset(fake_homed_cache_, 0, sizeof(fake_homed_cache_));
  fake_homed_cache_[0] = 1;

  //pyhtronCreateAxis uses portName to identify the controller
  this->controllerName_ = (char *) mallocMustSucceed(sizeof(char)*(strlen(portName)+1),
      "phytronController::phytronController: Controller name memory allocation failed.\n");

  strcpy(this->controllerName_, portName);

  //Create Controller parameters
  createParam(controllerStatusString,     asynParamInt32, &this->controllerStatus_);
  createParam(controllerStatusResetString,asynParamInt32, &this->controllerStatusReset_);
  createParam(resetControllerString,      asynParamInt32, &this->resetController_);

  //Create Axis parameters
  createParam(axisStatusResetString,      asynParamInt32,   &this->axisStatusReset_);
  createParam(axisResetString,            asynParamInt32,   &this->axisReset_);
  createParam(axisStatusString,           asynParamInt32,   &this->axisStatus_);
  createParam(homingProcedureString,      asynParamInt32,   &this->homingProcedure_);
  createParam(axisModeString,             asynParamInt32,   &this->axisMode_);
  createParam(mopOffsetPosString,         asynParamInt32,   &this->mopOffsetPos_);
  createParam(mopOffsetNegString,         asynParamInt32,   &this->mopOffsetNeg_);
  createParam(stepResolutionString,       asynParamInt32,   &this->stepResolution_);
  createParam(stopCurrentString,          asynParamInt32,   &this->stopCurrent_);
  createParam(runCurrentString,           asynParamInt32,   &this->runCurrent_);
  createParam(boostCurrentString,         asynParamInt32,   &this->boostCurrent_);
  createParam(encoderTypeString,          asynParamInt32,   &this->encoderType_);
  createParam(initRecoveryTimeString,     asynParamInt32,   &this->initRecoveryTime_);
  createParam(positionRecoveryTimeString, asynParamInt32,   &this->positionRecoveryTime_);
  createParam(boostConditionString,       asynParamInt32,   &this->boost_);
  createParam(encoderRateString,          asynParamInt32,   &this->encoderRate_);
  createParam(switchTypString,            asynParamInt32,   &this->switchTyp_);
  createParam(pwrStageModeString,         asynParamInt32,   &this->pwrStageMode_);
  createParam(encoderResolutionString,    asynParamInt32,   &this->encoderRes_);
  createParam(encoderFunctionString,      asynParamInt32,   &this->encoderFunc_);
  createParam(encoderSFIWidthString,      asynParamInt32,   &this->encoderSFIWidth_);
  createParam(encoderDirectionString,     asynParamInt32,   &this->encoderDirection_);
  createParam(powerStagetMonitorString,   asynParamInt32,   &this->powerStageMonitor_);
  createParam(currentDelayTimeString,     asynParamInt32,   &this->currentDelayTime_);
  createParam(powerStageTempString,       asynParamFloat64, &this->powerStageTemp_);
  createParam(motorTempString,            asynParamFloat64, &this->motorTemp_);
  createParam(axisBrakeOutputString,      asynParamFloat64, &this->axisBrakeOutput_);
  createParam(axisDisableMotorString,     asynParamInt32,   &this->axisDisableMotor_);
  createParam(axisBrakeEngageTimeString,  asynParamFloat64, &this->axisBrakeEngageTime_);
  createParam(axisBrakeReleaseTimeString, asynParamFloat64, &this->axisBrakeReleaseTime_);
  createParam(directCommandString,        asynParamOctet,   &this->directCommand_);
  createParam(directReplyString,          asynParamOctet,   &this->directReply_);
  createParam(directStatusString,         asynParamInt32,   &this->directStatus_);
  setStringParam(0, directReply_, "");
  setIntegerParam(0, directStatus_, 0);

  /* Connect to phytron controller */
  status = pasynOctetSyncIO->connect(szAsynPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
      "%s: cannot connect to phytron controller\n",
      functionName);
  } else {
    // this hook forces an initial readout before the motor record has
    // finished initializing (before pass 1), so it has actual values for
    // REP, RMP, MSTA fields, especially for absolute encoders
    if (controllers_.empty())
      initHookRegister(&phytronController::epicsInithookFunction);

    //phytronCreateAxis will search for the controller for axis registration
    controllers_.push_back(this);

    if (iNoResetAtBoot != 1) {
      epicsTimeStamp tStart, tNow;
      //RESET THE CONTROLLER
      if (sendPhytronCommand(std::string("CR")))
        asynPrint(this->pasynUserSelf, ASYN_TRACE_WARNING,
              "phytronController::phytronController: Could not reset controller %s\n", this->controllerName_);

      //Wait for reset to finish
      epicsThreadSleep(5.0);
      epicsTimeGetCurrent(&tStart);
      while (sendPhytronCommand(std::string("S")) != phytronSuccess)
      {
        epicsTimeGetCurrent(&tNow);
        if (epicsTimeDiffInSeconds(&tNow, &tStart) >= 120.)
        {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "phytronController::phytronController: no valid answer after controller reset %s\n", this->controllerName_);
          break;
        }
        epicsThreadSleep(0.5);
      }
    }

    if (iCtrlType == TYPE_MCC)
    {
      std::string sTmp;
      sendPhytronCommand(std::string("IAR"), sCtrlType_);
      sCtrlType_.insert(0, "MCC-");
      if (sendPhytronCommand(std::string("1P48R"), sTmp) == phytronSuccess)
      {
        long lType(-1);
        if (epicsParseLong(sTmp.c_str(), &lType, 0, NULL) != 0)
          lType = -1;
        switch (lType)
        {
          case 0: break; // MCC-1/MCC-2 chopper
          case 1: sCtrlType_.append(" LIN"); break; // MCC-1 linear
          default:
            sCtrlType_.append(" type");
            sCtrlType_.append(sTmp);
            break;
        }
      }
    }
    else
      sendPhytronCommand(std::string("IM0"), sCtrlType_);
    sCtrlType_ = escapeC(sCtrlType_);

    startPoller(iMovingPollPeriod, iIdlePollPeriod, 5);
  }
}

/** Creates a new phytronController object for a Phytron phyMOTION.
  * Configuration command, called directly or from iocsh
  * \param[in] szPhytronPortName  The name of the asyn port that will be created for this driver
  * \param[in] szAsynPortName     The name of the drvAsynIPPPort that was created previously to connect to the phytron controller
  * \param[in] iMovingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] iIdlePollPeriod    The time in ms between polls when no axis is moving
  * \param[in] dTimeout           Communication timeout in ms
  * \param[in] iNoResetAtBoot     if 0 or not set then controller is reset, if 1 then it's not
  */
int phytronController::phytronCreatePhymotion(const char* szPhytronPortName,
    const char* szAsynPortName, int iMovingPollPeriod, int iIdlePollPeriod,
    double dTimeout, int iNoResetAtBoot)
{
  new phytronController(phytronController::TYPE_PHYMOTION, szPhytronPortName, szAsynPortName, 0,
                        iMovingPollPeriod/1000., iIdlePollPeriod/1000., dTimeout/1000., iNoResetAtBoot);
  return asynSuccess;
}

/** Creates a new phytronController object for a Phytron MCC-1 or MCC-2.
  * Configuration command, called directly or from iocsh
  * \param[in] szPhytronPortName  The name of the asyn port that will be created for this driver
  * \param[in] szAsynPortName     The name of the drvAsynIPPPort that was created previously to connect to the phytron controller
  * \param[in] iAddress           Serial address of contoller 0..15 (hardware selector)
  * \param[in] iMovingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] iIdlePollPeriod    The time in ms between polls when no axis is moving
  * \param[in] dTimeout           Communication timeout in ms
  * \param[in] iNoResetAtBoot     if 0 or not set then controller is reset, if 1 then it's not
  */
int phytronController::phytronCreateMCC(const char* szPhytronPortName,
    const char* szAsynPortName, int iAddress, int iMovingPollPeriod,
    int iIdlePollPeriod, double dTimeout, int iNoResetAtBoot)
{
  new phytronController(phytronController::TYPE_MCC, szPhytronPortName, szAsynPortName, iAddress,
                        iMovingPollPeriod/1000., iIdlePollPeriod/1000., dTimeout/1000., iNoResetAtBoot);
  return asynSuccess;
}

/** asynUsers use this to read integer parameters
 * \param[in]  pasynUser  asynUser structure containing the reason
 * \param[out] piValue    Parameter value
 */
asynStatus phytronController::readInt32(asynUser* pasynUser, epicsInt32* piValue)
{
  phytronAxis   *pAxis;
  phytronStatus phyStatus;
  std::string   sResponse;
  int           iParameter(0);

  //Call base implementation first
  asynMotorController::readInt32(pasynUser, piValue);

  //Check if this is a call to read a controller parameter
  if(pasynUser->reason == resetController_ || pasynUser->reason == controllerStatusReset_){
    //Called only on initialization of bo records RESET and RESET-STATUS
    return asynSuccess;
  } else if (pasynUser->reason == controllerStatus_){
    phyStatus = sendPhytronCommand(std::string("ST"), sResponse);
    CHECK_CTRL("readInt32", "Reading controller status", return phyToAsyn(phyStatus));

    *piValue = atoi(sResponse.c_str());
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
    getIntegerParam(pAxis->axisNo_, homingProcedure_, piValue);
    return asynSuccess;
  } else if (pasynUser->reason == axisReset_ || pasynUser->reason == axisStatusReset_){
    //Called only on initialization of AXIS-RESET and AXIS-STATUS-RESET bo records
    return asynSuccess;
  } else if (pasynUser->reason == axisDisableMotor_) {
    *piValue = pAxis->disableMotor_ & 1;
    return asynSuccess;
  }
  else if (pasynUser->reason == axisMode_)             iParameter =  1;
  else if (pasynUser->reason == mopOffsetPos_)         iParameter = 11;
  else if (pasynUser->reason == mopOffsetNeg_)         iParameter = 12;
  else if (pasynUser->reason == stepResolution_)       iParameter = 45;
  else if (pasynUser->reason == stopCurrent_)          iParameter = 40;
  else if (pasynUser->reason == runCurrent_)           iParameter = 41;
  else if (pasynUser->reason == boostCurrent_)         iParameter = 42;
  else if (pasynUser->reason == encoderType_)          iParameter = 34;
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

  phyStatus = sendPhytronCommand(std::string(pAxis->axisModuleNo_) + "P" + std::to_string(iParameter) + "R",
                                 sResponse);
  CHECK_AXIS("readInt32", "Reading parameter", pAxis, return phyToAsyn(phyStatus));

  *piValue = atoi(sResponse.c_str());

  //{STOP,RUN,BOOST} current records have EGU set to mA, but device returns 10mA
  if(pasynUser->reason == stopCurrent_ || pasynUser->reason == runCurrent_ ||
      pasynUser->reason == boostCurrent_)
    *piValue *= 10;

  return asynSuccess;
}

/** asynUsers use this to write integer parameters
 * \param[in] pasynUser  asynUser structure containing the reason
 * \param[in] iValue     Parameter value to be written
 */
asynStatus phytronController::writeInt32(asynUser* pasynUser, epicsInt32 iValue)
{
  phytronAxis   *pAxis;
  phytronStatus phyStatus(phytronSuccess);

  //Call base implementation first
  asynMotorController::writeInt32(pasynUser, iValue);
  const char* szSetChar((iCtrlType_ == TYPE_PHYMOTION) ? "=" : "S");

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
    setIntegerParam(pAxis->axisNo_, pasynUser->reason, iValue);
    callParamCallbacks();
    return asynSuccess;
  } else if(pasynUser->reason == axisReset_){
    sprintf(this->outString_, "%sC", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == axisStatusReset_){
    if (iCtrlType_ != TYPE_PHYMOTION)
      return asynError;
    sprintf(this->outString_, "SEC%s", &pAxis->axisModuleNo_[1]);
  } else if(pasynUser->reason == axisMode_){
    sprintf(this->outString_, "%sP01%s%d", pAxis->axisModuleNo_, szSetChar,iValue);
  } else if(pasynUser->reason == mopOffsetPos_){
    sprintf(this->outString_, "%sP11%s%d", pAxis->axisModuleNo_, szSetChar,iValue);
  } else if(pasynUser->reason == mopOffsetNeg_){
    sprintf(this->outString_, "%sP12%s%d", pAxis->axisModuleNo_, szSetChar,iValue);
  } else if (pasynUser->reason == stepResolution_){
    sprintf(this->outString_, "%sP45%s%d", pAxis->axisModuleNo_, szSetChar,iValue);
  }  else if (pasynUser->reason == stopCurrent_){
    iValue /= 10; //STOP_CURRENT record has EGU mA, device expects 10mA
    sprintf(this->outString_, "%sP40%s%d", pAxis->axisModuleNo_, szSetChar,iValue);
  } else if (pasynUser->reason == runCurrent_){
    iValue /= 10; //RUN_CURRENT record has EGU mA, device expects 10mA
    sprintf(this->outString_, "%sP41%s%d", pAxis->axisModuleNo_, szSetChar,iValue);
  } else if (pasynUser->reason == boostCurrent_){
    iValue /= 10; //BOOST_CURRENT record has EGU mA, device expects 10mA
    sprintf(this->outString_, "%sP42%s%d", pAxis->axisModuleNo_, szSetChar,iValue);
  } else if (pasynUser->reason == encoderType_){
    sprintf(this->outString_, "%sP34%s%d", pAxis->axisModuleNo_, szSetChar, iValue);
  } else if (pasynUser->reason == initRecoveryTime_){
    sprintf(this->outString_, "%sP13%s%d", pAxis->axisModuleNo_, szSetChar, iValue);
  } else if (pasynUser->reason == positionRecoveryTime_){
    sprintf(this->outString_, "%sP16%s%d", pAxis->axisModuleNo_, szSetChar, iValue);
  } else if (pasynUser->reason == boost_){
    sprintf(this->outString_, "%sP17%s%d", pAxis->axisModuleNo_, szSetChar, iValue);
  } else if (pasynUser->reason == encoderRate_){
    sprintf(this->outString_, "%sP26%s%d", pAxis->axisModuleNo_, szSetChar, iValue);
  } else if (pasynUser->reason == switchTyp_){
    sprintf(this->outString_, "%sP27%s%d", pAxis->axisModuleNo_, szSetChar, iValue);
  } else if (pasynUser->reason == pwrStageMode_){
    sprintf(this->outString_, "%sP28%s%d", pAxis->axisModuleNo_, szSetChar, iValue);
  } else if (pasynUser->reason == encoderRes_){
    sprintf(this->outString_, "%sP35%s%d", pAxis->axisModuleNo_, szSetChar, iValue);
  } else if (pasynUser->reason == encoderFunc_){
    //Value is VAL field of parameter P37 record. If P37 is positive P36 is set to 1, else 0
    sprintf(this->outString_, "%sP36%s%d", pAxis->axisModuleNo_, szSetChar, iValue > 0 ? 1 : 0);
  } else if(pasynUser->reason == encoderSFIWidth_){
    sprintf(this->outString_, "%sP37%s%d", pAxis->axisModuleNo_, szSetChar, iValue);
  } else if(pasynUser->reason == encoderSFIWidth_){
    sprintf(this->outString_, "%sP38%s%d", pAxis->axisModuleNo_, szSetChar, iValue);
  } else if(pasynUser->reason == powerStageMonitor_){
    sprintf(this->outString_, "%sP53%s%d", pAxis->axisModuleNo_, szSetChar, iValue);
  } else if(pasynUser->reason == currentDelayTime_){
    sprintf(this->outString_, "%sP43%s%d", pAxis->axisModuleNo_, szSetChar, iValue);
  } else if(pasynUser->reason == encoderDirection_){
    sprintf(this->outString_, "%sP38%s%d", pAxis->axisModuleNo_, szSetChar, iValue);
  } else if(pasynUser->reason == axisDisableMotor_){
    pAxis->disableMotor_ = (pAxis->disableMotor_ & (~1)) | ((iValue != 0) ? 1 : 0);
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
 * \param[in]  pasynUser  asynUser structure containing the reason
 * \param[out] pdValue    Parameter value
 */
asynStatus phytronController::readFloat64(asynUser *pasynUser, epicsFloat64 *pdValue)
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
  asynMotorController::readFloat64(pasynUser, pdValue);

  this->outString_[0] = '\0';
  if(pasynUser->reason == powerStageTemp_){
    sprintf(this->outString_, "%sP49R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == motorTemp_){
    sprintf(this->outString_, "%sP54R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == axisBrakeOutput_){
    *pdValue = pAxis->brakeOutput_;
    return asynSuccess;
  } else if(pasynUser->reason == axisBrakeEngageTime_){
    *pdValue = pAxis->brakeEngageTime_;
    return asynSuccess;
  } else if(pasynUser->reason == axisBrakeReleaseTime_){
    *pdValue = pAxis->brakeReleaseTime_;
    return asynSuccess;
  }

  if (this->outString_[0]){
    phyStatus = sendPhytronCommand(const_cast<const char*>(this->outString_), sResponse);
    CHECK_AXIS("readFloat64", "Reading parameter", pAxis, return phyToAsyn(phyStatus));

    //Power stage and motor temperature records have EGU °C, but device returns 0.1 °C
    *pdValue = atof(sResponse.c_str()) / 10.;
  }

  return asynSuccess;
}

/** asynUsers use this to write float parameters
 * \param[in] pasynUser  asynUser structure containing the reason
 * \param[in] dValue     Parameter value to be written
 */
asynStatus phytronController::writeFloat64(asynUser* pasynUser, epicsFloat64 dValue)
{
  phytronAxis* pAxis(getAxis(pasynUser));
  if(!pAxis){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
       "phytronAxis::writeFloat64: Axis not found on the controller %s\n", this->controllerName_);
    return asynError;
  }

  if (pasynUser->reason == axisBrakeOutput_)
    pAxis->brakeOutput_ = floor(10.0 * ((dValue > -100.0 && dValue < 100.) ? dValue : 0.0) + 0.5) / 10.0;
  else if (pasynUser->reason == axisBrakeEngageTime_) {
    if (dValue > MAXIMUM_BRAKE_TIME)
      dValue = MAXIMUM_BRAKE_TIME;
    pAxis->brakeEngageTime_ = dValue > 0.0 ? dValue : 0.0;
  } else if (pasynUser->reason == axisBrakeReleaseTime_) {
    if (dValue > MAXIMUM_BRAKE_TIME)
      dValue = MAXIMUM_BRAKE_TIME;
    pAxis->brakeReleaseTime_ = dValue > 0.0 ? dValue : 0.0;
  }

  //Call base implementation
  return asynMotorController::writeFloat64(pasynUser, dValue);
}

/** Called when asyn clients call pasynOctet->write().
 * \param[in]  pasynUser  asynUser structure containing the reason
 * \param[in]  szValue    string to write
 * \param[in]  maxChars   number of characters to write
 * \param[out] pnActual   number of characters actually written
 */
asynStatus phytronController::writeOctet(asynUser* pasynUser, const char* szValue, size_t maxChars, size_t* pnActual)
{
  asynStatus iResult(asynSuccess);
  std::string sCmd(szValue, maxChars), sReply;
  if (pasynUser->reason == directCommand_)
  {
    iResult = phyToAsyn(sendPhytronCommand(sCmd, sReply, false));
    if (iResult == asynSuccess)
    {
      setStringParam(0, directReply_, "");
      setIntegerParam(0, directStatus_, 0);
      if      (sReply.substr(0, 1) == "\x06") setIntegerParam(0, directStatus_, 1);
      else if (sReply.substr(0, 1) == "\x15") setIntegerParam(0, directStatus_, 2);
      else
      {
        setIntegerParam(0, directStatus_, 3);
        goto noerase;
      }
      sReply.erase(0, 1);
noerase:
      setStringParam(0, directReply_, sReply);
    }
  }
  else if (pasynUser->reason == directReply_ || pasynUser->reason == directStatus_)
    iResult = asynError;
  if (iResult == asynSuccess)
    iResult = asynPortDriver::writeOctet(pasynUser, szValue, maxChars, pnActual);
  return iResult;
}

/** Called when asyn clients call pasynOption->read().
 * The base class implementation simply prints an error message.
 * Derived classes may reimplement this function if required.
 * \param[in]  pasynUser  pasynUser structure that encodes the reason and address.
 * \param[in]  szKey      Option key string.
 * \param[out] szValue    string to be returned
 * \param[in]  iMaxChars  Size of value string
 * \return asyn status
 */
asynStatus phytronController::readOption(asynUser* pasynUser, const char* szKey, char* szValue, int iMaxChars)
{
  if (szKey && szValue) {
    *szValue = '\0';
    if (epicsStrCaseCmp(szKey, "pollMethod") == 0) {
      // polling methods: serial, axis-parallel or controller-parallel; axis may have "default"
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
      snprintf(szValue, iMaxChars, "%d/%s", static_cast<int>(iMethod), szMethod);
      return asynSuccess;
    }

    if (epicsStrCaseCmp(szKey, "clearAxisStatus") == 0) {
      // "clear axis status" before move
      phytronAxis* pAxis(getAxis(pasynUser));
      float fTime(fabs(pAxis ? pAxis->statusResetTime_ : statusResetTime_));
      if (!isfinite(fTime))
        snprintf(szValue, iMaxChars, "default");
      else if (fTime >= 0.001)
        snprintf(szValue, iMaxChars, "%g", fTime);
      else if (fTime > 0)
        snprintf(szValue, iMaxChars, "on");
      else
        snprintf(szValue, iMaxChars, "off");
      return asynSuccess;
    }

    if (epicsStrCaseCmp(szKey, "fakeHomedEnable") == 0) {
      // configure "fake homed" bit
      snprintf(szValue, iMaxChars, "%s", fake_homed_enable_ ? "true" : "false");
      return asynSuccess;
    }

    if (epicsStrCaseCmp(szKey, "fakeHomedCache") == 0) {
      // show "fake homed" cache
      int iLen(0);
      szValue[0] = '\0';
      for (int i = 0; i < ARRAY_SIZE(fake_homed_cache_); ++i) {
        snprintf(&szValue[iLen], iMaxChars - iLen, "%s%u", i ? "," : "", fake_homed_cache_[i]);
        szValue[iMaxChars - 1] = '\0';
        iLen += strlen(&szValue[iLen]);
        if ((iLen + 1) >= iMaxChars)
          break;
      }
      return asynSuccess;
    }

    if (epicsStrCaseCmp(szKey, "allowExitOnError") == 0) {
      // configure "allow exit on error": after detection of this, the IOC exists and should be restarted by procServ
      snprintf(szValue, iMaxChars, "%s", allow_exit_on_error_ ? "true" : "false");
      return asynSuccess;
    }
  }
  return asynMotorController::readOption(pasynUser, szKey, szValue, iMaxChars);
}

/** Called when asyn clients call pasynOption->write().
 * The base class implementation simply prints an error message.
 * Derived classes may reimplement this function if required.
 * \param[in] pasynUser  pasynUser structure that encodes the reason and address.
 * \param[in] szKey      Option key string.
 * \param[in] szValue    Value string.
 * \return asyn status
 */
asynStatus phytronController::writeOption(asynUser* pasynUser, const char* szKey, const char* szValue)
{
  phytronAxis* pAxis(NULL);
  enum pollMethod iMethod(pollMethodSerial);
  epicsInt64 iTmp(-2);
  int iAxisNo(0);
  size_t iLen;

  if (!szKey || !szValue)
    goto finish;
  if (epicsStrCaseCmp(szKey, "pollMethod") == 0) {
    // polling methods: serial, axis-parallel or controller-parallel; axis may have "default"
    getAddress(pasynUser, &iAxisNo);
    if (iAxisNo > 0) {
      pAxis = getAxis(iAxisNo);
      if (!pAxis) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "phytronController:writeOption(%s, %s) wrong axis", szKey, szValue);
        return asynError;
      }
      iMethod = pollMethodDefault;
    }
    if (epicsParseInt64(szValue, &iTmp, 0, NULL) == 0) {
      if (iTmp < (pAxis ? static_cast<epicsInt64>(pollMethodDefault) : static_cast<epicsInt64>(pollMethodSerial)) ||
          iTmp > static_cast<epicsInt64>(pollMethodControllerParallel))
        goto wrongValue_parallel;
      iMethod = static_cast<pollMethod>(iTmp);
    } else {
      while (isspace(*szValue)) ++szValue;
      iLen = strlen(szValue);
      while (iLen > 0 && isspace(szValue[iLen - 1])) --iLen;

      if (pAxis && ((iLen ==  7 && !epicsStrnCaseCmp(szValue, "default", 7)) ||
                    (iLen ==  8 && !epicsStrnCaseCmp(szValue, "standard", 8)) ||
                    (iLen == 10 && !epicsStrnCaseCmp(szValue, "controller", 10))))
        iMethod = pollMethodDefault;
      else if ((iLen ==  6 && !epicsStrnCaseCmp(szValue, "serial", 6)) ||
               (iLen == 11 && !epicsStrnCaseCmp(szValue, "no-parallel", 11)) ||
               (iLen == 12 && !epicsStrnCaseCmp(szValue, "not-parallel", 12)) ||
               (iLen ==  3 && !epicsStrnCaseCmp(szValue, "old", 3)))
        iMethod = pollMethodSerial;
      else if ((iLen ==  4 && !epicsStrnCaseCmp(szValue, "axis", 4)) ||
               (iLen == 13 && !epicsStrnCaseCmp(szValue, "axis-parallel", 13)))
        iMethod = pollMethodAxisParallel;
      else if ((iLen ==  4 && !epicsStrnCaseCmp(szValue, "ctrl", 4)) ||
               (iLen ==  8 && !epicsStrnCaseCmp(szValue, "parallel", 8)) ||
               (iLen == 13 && !epicsStrnCaseCmp(szValue, "ctrl-parallel", 13)) ||
               (iLen == 19 && !epicsStrnCaseCmp(szValue, "controller-parallel", 19)))
        iMethod = pollMethodControllerParallel;
      else
        goto wrongValue_parallel;
    }
    if (iCtrlType_ != TYPE_PHYMOTION && iMethod != pollMethodDefault && iMethod != pollMethodSerial)
    {
      epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                    "phytronController:writeOption(%s, %s) pollMethod not allowed for this controller", szKey, szValue);
      return asynError;
    }
    if (pAxis)
      pAxis->iPollMethod_ = iMethod;
    else
      iDefaultPollMethod_ = iMethod;
    return asynSuccess;
  }

  if (epicsStrCaseCmp(szKey, "clearAxisStatus") == 0) {
    // configure "clear axis status" before move
    float fTime(0.), *pfTime(&statusResetTime_);
    getAddress(pasynUser, &iAxisNo);
    if (iAxisNo > 0) {
      pAxis = getAxis(iAxisNo);
      if (!pAxis) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "phytronController:writeOption(%s, %s) wrong axis", szKey, szValue);
        return asynError;
      }
      pfTime = &pAxis->statusResetTime_;
    }

    if (epicsParseFloat(szValue, &fTime, NULL) == 0 && fTime >= 0. && fTime <= 10.)
      ;
    else if (pAxis && epicsStrCaseCmp(szValue, "default") == 0)
      fTime = epicsINF;
    else if (epicsStrCaseCmp(szValue, "t") == 0 || epicsStrCaseCmp(szValue, "true") == 0 ||
             epicsStrCaseCmp(szValue, "y") == 0 || epicsStrCaseCmp(szValue, "yes") == 0 ||
             epicsStrCaseCmp(szValue, "on") == 0)
      fTime = 0.000001;
    else if (epicsStrCaseCmp(szValue, "f") == 0 || epicsStrCaseCmp(szValue, "false") == 0 ||
             epicsStrCaseCmp(szValue, "n") == 0 || epicsStrCaseCmp(szValue, "no") == 0 ||
             epicsStrCaseCmp(szValue, "off") == 0)
      fTime = 0.;
    else {
      epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                    "phytronController:writeOption(%s, %s) wrong value - allowed is seconds (0..10) or boolean%s",
                    szKey, szValue, pAxis ? " or \"default\"" : "");
      goto wrongValue;
    }
    *pfTime = (*pfTime <= 0.) ? -fTime : fTime;
    return asynSuccess;
  }

  if (epicsStrCaseCmp(szKey, "fakeHomedEnable") == 0) {
    // configure "fake homed" bit: ORed real homed status with autosave position
    if (epicsParseInt64(szValue, &iTmp, 0, NULL) == 0)
      fake_homed_enable_ = (iTmp != 0);
    else if (epicsStrCaseCmp(szValue, "t") == 0 || epicsStrCaseCmp(szValue, "true") == 0 ||
             epicsStrCaseCmp(szValue, "y") == 0 || epicsStrCaseCmp(szValue, "yes") == 0 ||
             epicsStrCaseCmp(szValue, "on") == 0)
    {
      const char* szSetChar((iCtrlType_ == TYPE_PHYMOTION) ? "=" : "S");
      if (sendPhytronCommand(std::string("R1001") + szSetChar + std::to_string(fake_homed_cache_[0])) != phytronSuccess)
        return asynError;
      fake_homed_enable_ = true;
    }
    else if (epicsStrCaseCmp(szValue, "f") == 0 || epicsStrCaseCmp(szValue, "false") == 0 ||
             epicsStrCaseCmp(szValue, "n") == 0 || epicsStrCaseCmp(szValue, "no") == 0 ||
             epicsStrCaseCmp(szValue, "off") == 0)
      fake_homed_enable_ = false;
    else
      goto wrongValue_bool;
    return asynSuccess;
  }

  if (epicsStrCaseCmp(szKey, "allowExitOnError") == 0) {
    // configure "allow exit on error": after detection of this, the IOC exists and should be restarted by procServ
    if (epicsParseInt64(szValue, &iTmp, 0, NULL) == 0)
      allow_exit_on_error_ = (iTmp != 0);
    else if (epicsStrCaseCmp(szValue, "t") == 0 || epicsStrCaseCmp(szValue, "true") == 0 ||
             epicsStrCaseCmp(szValue, "y") == 0 || epicsStrCaseCmp(szValue, "yes") == 0 ||
             epicsStrCaseCmp(szValue, "on") == 0)
      allow_exit_on_error_ = true;
    else if (epicsStrCaseCmp(szValue, "f") == 0 || epicsStrCaseCmp(szValue, "false") == 0 ||
             epicsStrCaseCmp(szValue, "n") == 0 || epicsStrCaseCmp(szValue, "no") == 0 ||
             epicsStrCaseCmp(szValue, "off") == 0)
      allow_exit_on_error_ = false;
    else
      goto wrongValue_bool;
    return asynSuccess;
  }

finish:
  return asynMotorController::writeOption(pasynUser, szKey, szValue);

wrongValue_parallel:
  epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "phytronController:writeOption(%s, %s) wrong value - allowed are %sserial,axis-parallel,%sparallel",
                szKey, szValue, pAxis ? "default," : "", pAxis ? "controller-" : "");
  goto wrongValue;

wrongValue_bool:
  epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "phytronController:writeOption(%s, %s) wrong value - allowed is boolean", szKey, szValue);

wrongValue:
  return asynError;
}

/**
 * Reset the motorEncoderRatio to 1 after the reset of MCM unit
 */
void phytronController::resetAxisEncoderRatio()
{
  for(uint32_t i = 0; i < axes_.size(); i++){
    setDoubleParam(axes_[i]->axisNo_, motorEncoderRatio_, 1);
  }
  callParamCallbacks();
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
 * - phyMOTION
 *   4 single command request and replies: motor position, encoder position,
 *   moving status, axis status (limit switches, home switches, ...),
 *   takes about 40ms per axis
 * - MCC
 *   4+ single command request and replies: motor position, encoder position,
 *   moving status, error status and plus one limit switches in controller poll
 * * pollMethodAxisParallel or
 * * pollMethodControllerParallel:
 * - phyMOTION only
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
    std::vector<std::string> asCommands, asResponses;
    if (fake_homed_enable_ > 0)
      asCommands.push_back(std::string("R1001R"));
    switch (iCtrlType_)
    {
      case TYPE_PHYMOTION:
        for (std::vector<phytronAxis*>::iterator it = axes_.begin(); it != axes_.end(); ++it) {
          phytronAxis* pAxis(*it);
          if (!pAxis) continue;
          pollMethod iPollMethod(pAxis->iPollMethod_);
          if (iPollMethod < 0) iPollMethod = iDefaultPollMethod_;
          // this axis is configured for controller parallel poll and handled here
          if (iPollMethod == pollMethodControllerParallel)
          {
            apTodoAxis.emplace_back(std::move(pAxis));
            pAxis->appendRequestString(asCommands); // collect requests
          }
        }
        break;
      case TYPE_MCC:
        sLastSUI_.clear();
        asCommands.push_back(std::string("SUI"));
        break;
      default:
        return asynError;
    }
    for (std::vector<phytronIO*>::iterator it = IOs_.begin(); it != IOs_.end(); ++it)
      (*it)->pollIO(asCommands, true);
    if (!asCommands.empty())
    {
      // communicate
      iResult = phyToAsyn(sendPhytronMultiCommand(asCommands, asResponses, true, iCtrlType_ != TYPE_PHYMOTION));
      if (iResult == asynSuccess)
      {
        // handle answers
        while (fake_homed_enable_)
        {
          double dTmp(static_cast<double>(epicsNAN));
          const char* szValue;
          if (asCommands.empty())
          {
            iResult = asynError;
            break;
          }
          szValue = asResponses.front().c_str();
          if (*szValue++ != '\x06') // need ACK here
            iResult = asynError;
          if (epicsParseDouble(szValue, &dTmp, NULL) != 0)
            iResult = asynError;
          else if (!isfinite(dTmp) || floor(dTmp + 0.5) < 0. || floor(dTmp + 0.5) >= 65536.)
            iResult = asynError;
          else if (static_cast<epicsUInt64>(floor(dTmp + 0.5)) != fake_homed_cache_[0])
          {
            // detected a difference in cached value, the Phytron was restarted
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "detected a difference in cached fake-homed-bit value, did the Phytron a restart?\n");
            if (allow_exit_on_error_)
              epicsExit(1);
            fake_homed_cache_[0] = static_cast<epicsUInt64>(floor(dTmp + 0.5));
          }
          asResponses.erase(asResponses.begin());
          break;
        }
        if (iCtrlType_ == TYPE_MCC)
        {
          if (!asResponses.empty())
          {
            sLastSUI_ = asResponses.front();
            asResponses.erase(asResponses.begin());
            if (sLastSUI_.empty())
              iResult = asynError;
            else if (sLastSUI_.front() != '\x06')
              iResult = asynError;
            else
              sLastSUI_.erase(0, 1);
          }
          else
            iResult = asynError;
        }
        else if (!apTodoAxis.empty()) // iCtrlType_ == TYPE_PHYMOTION
          for (std::vector<phytronAxis*>::iterator it = apTodoAxis.begin(); it != apTodoAxis.end(); ++it)
            if (!(*it)->parseAnswer(asResponses))
              iResult = asynError;
        for (std::vector<phytronIO*>::iterator it = IOs_.begin(); it != IOs_.end(); ++it)
          if (!(*it)->pollIO(asResponses, false))
            iResult = asynError;
        if (!asResponses.empty())
          iResult = asynError;
      }
      else if (!apTodoAxis.empty())
        for (std::vector<phytronAxis*>::iterator it = axes_.begin(); it != axes_.end(); ++it)
        {
          (*it)->setIntegerParam(motorStatusProblem_, 1); // set problem bit
          (*it)->callParamCallbacks();
        }
    }
  }
  if (iResult != asynSuccess)
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "phytronController::poll(%s) error %d\n", controllerName_, iResult);
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
  for (vector<phytronController*>::iterator itC = controllers_.begin();
       itC != controllers_.end(); ++itC) {
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
  epicsUInt8 byCRC(0), byCRC2(0);
  if (sCommand.size() > 994) // maximum command length exceeded (1000 bytes incl. framing)
    return phytronInvalidCommand;
  if (sCommand.capacity() < sCommand.size() + 6)
    sCommand.reserve(sCommand.size() + 6);
  if (iAddress_ >= 0 && iAddress_ <= 15)
  {
    // address of controller
    char szAddr[32];
    epicsSnprintf(szAddr, sizeof(szAddr), "%X", iAddress_);
    szAddr[2] = '\0';
    sCommand.insert(0, const_cast<const char*>(&szAddr[0]));
  }
  else
    sCommand.insert(0, "0"); // default address of controller
  // put framing
  if (iCtrlType_ == TYPE_PHYMOTION) // phyMOTION with CRC
  {
    sCommand.push_back(':'); // separator for CRC
    for (auto it = sCommand.begin(); it != sCommand.end(); ++it)
      byCRC ^= static_cast<epicsUInt8>(*it); // "calculate" CRC
    byCRC2 = byCRC & 0x0F;
    byCRC >>= 4;
    sCommand.push_back(static_cast<char>(byCRC  + ((byCRC  > 9) ? ('A' - 10) : '0')));
    sCommand.push_back(static_cast<char>(byCRC2 + ((byCRC2 > 9) ? ('A' - 10) : '0')));
  }
  sCommand.insert(0, "\x02"); // STX
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
        if (!bACKonly)
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
 * \return asynStatus
 */
asynStatus phytronController::phyToAsyn(phytronStatus phyStatus)
{
  if(phyStatus == phytronInvalidReturn || phyStatus == phytronInvalidCommand) return asynError;
  return (asynStatus) phyStatus;
}

/**
 * Converts non-printable characters to C-escape notation
 * \param[in] sIn  text to convert
 * \return converted printable text
 */
std::string phytronController::escapeC(std::string sIn)
{
  std::string sOut;
  sOut.resize(4 * sIn.size() + 10);
  sOut.resize(epicsStrnEscapedFromRaw(&sOut[0], sOut.size(), sIn.c_str(), sIn.size()));
  return sOut;
}


//******************************************************************************
//                   PHYTRON AXIS IMPLEMENTATION
//******************************************************************************

/** Creates a new phytronAxis object.
  * Configuration command, called directly or from iocsh
  * \param[in] szControllerName   Name of the asyn port created by calling phytronCreatePhymotion or phytronCreateMCC from st.cmd
  * \param[in] iModule            Index of the I1AM01 module controlling this axis
  * \param[in] iAxis              Axis index
  */
int phytronAxis::phytronCreateAxis(const char* szControllerName, int iModule, int iAxis)
{
  phytronAxis *pAxis;
  uint32_t i;

  if (iAxis <= 0) {
    printf("ERROR: invalid axis specified\n");
    return asynError;
  }

  //Find the controller
  for(i = 0; i < phytronController::controllers_.size(); i++){
    phytronController* pC(phytronController::controllers_[i]);
    if(!strcmp(pC->controllerName_, szControllerName)) {
      pC->lock();
      pAxis = new phytronAxis(pC, iModule*10 + iAxis);
      pC->axes_.push_back(pAxis);
      pC->unlock();
      break;
    }
  }

  //If controller is not found, report error
  if(i == phytronController::controllers_.size()){
    printf("ERROR: phytronCreateAxis: Controller %s is not registered\n", szControllerName);
    return asynError;
  }

  return asynSuccess;
}

/** Configures a phytronAxis object to use a brake
 * \param[in] szControllerName    Name of the asyn port created by calling phytronCreatePhymotion or phytronCreateMCC from st.cmd
 * \param[in] fAxis               axis index (module.index)
 * \param[in] fOutput             digital out index (module.index) or 0 to disable this function, negative value inverts output
 * \param[in] bDisableMotor       0=keep motor enabled, 1=disable idle motor/enable when moved
 * \param[in] dEngageTime         time to engage brake (disable motor after this time in milliseconds, max. 10 sec)
 * \param[in] dReleaseTime        time to release brake (start move after this time in milliseconds, max. 10 sec)
 */
int phytronAxis::phytronBrakeOutput(const char* szControllerName, float fAxis, float fOutput, int bDisableMotor, double dEngageTime, double dReleaseTime)
{
  char szAxisNo[5];
  uint32_t i, j;
  if (!szControllerName || !*szControllerName)
    goto failed;
  for (i = 0; i < phytronController::controllers_.size(); ++i) {
    phytronController* pC(phytronController::controllers_[i]);
    if (!pC || strcmp(pC->controllerName_, szControllerName) != 0)
      continue;
    // found controller, search axis
    switch (pC->iCtrlType_)
    {
      case phytronController::TYPE_PHYMOTION:
        epicsSnprintf(szAxisNo, sizeof(szAxisNo), "M%.1f", fAxis);
        break;
      case phytronController::TYPE_MCC:
      {
        int iAxis(static_cast<int>(floor(10. * fAxis + .5)));
        if (iAxis < 1  || iAxis > 8)
          continue;
        epicsSnprintf(szAxisNo, sizeof(szAxisNo), "%d", iAxis);
        break;
      }
      default:
        continue;
    }
    szAxisNo[sizeof(szAxisNo) - 1] = '\0';
    for (j = 0; j < pC->axes_.size(); ++j) {
      phytronAxis* pA(pC->axes_[j]);
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
  * \param[in] iAxisNo Index number of this axis, range 0 to pC->numAxes_-1.
  *
  * Initializes register numbers, etc.
  */
phytronAxis::phytronAxis(phytronController *pC, int iAxisNo)
  : asynMotorAxis(pC, iAxisNo)
  , brakeOutput_(0.0)
  , disableMotor_(0)
  , brakeEngageTime_(0.0)
  , brakeReleaseTime_(0.0)
  , pC_(pC)
  , lastStatus_(phytronSuccess)
  , brakeReleased_(0)
  , iPollMethod_(pollMethodDefault)
  , homeState_(0)
  , statusResetTime_(epicsINF)
{
  //Controller always supports encoder. Encoder enable/disable is set through UEIP
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);
  setDoubleParam(pC_->motorEncoderRatio_, 1);
  axisModuleNo_[0] = '\0';
  switch (pC_->iCtrlType_)
  {
    case phytronController::TYPE_PHYMOTION:
      epicsSnprintf(axisModuleNo_, sizeof(axisModuleNo_), "M%.1f", iAxisNo / 10.);
      break;
    case phytronController::TYPE_MCC:
    {
      iAxisNo %= 10;
      if (iAxisNo >= 1 && iAxisNo <= 8)
        epicsSnprintf(axisModuleNo_, sizeof(axisModuleNo_), "%d", iAxisNo);
      break;
    }
  }
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
  if (level > 0)
    fprintf(fp, "  axis %d\n", axisNo_);

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

/**
 * Clears axis errors before next try to move.
 * \param[in,out] pCmdList  list of commands to append to or NULL (execute directly)
 * \return communication status
 */
phytronStatus phytronAxis::clearAxisError(std::vector<std::string>* pCmdList)
{
  float fTime(statusResetTime_);
  if (pC_->iCtrlType_ != phytronController::TYPE_PHYMOTION )
    return phytronSuccess; // not supported
  if (!isfinite(fTime) && fTime > 0.)
    fTime = fabs(pC_->statusResetTime_);
  if (fTime <= 0.0)
    return phytronSuccess; // not needed
  if (fTime > 10.)
    fTime = 10.;
  statusResetTime_ = -statusResetTime_;
  if (fTime >= 0.001) // reset status and wait
  {
    phytronStatus iResult(pC_->sendPhytronCommand(std::string("SEC") + &axisModuleNo_[1]));
    epicsThreadSleep(fTime);
    return iResult;
  }
  if (pCmdList)
  {
    pCmdList->push_back(std::string("SEC") + &axisModuleNo_[1]);
    return phytronSuccess;
  }
  else
    return pC_->sendPhytronCommand(std::string("SEC") + &axisModuleNo_[1]);
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
  std::vector<std::string> asCommands;
  enum pollMethod iPollMethod(iPollMethod_);
  const char* szSetChar((pC_->iCtrlType_ == phytronController::TYPE_PHYMOTION) ? "=" : "S");
  if (iPollMethod < 0) iPollMethod = pC_->iDefaultPollMethod_;
  if (!pCmdList) pCmdList = &asCommands;

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
      pCmdList->push_back(std::string(axisModuleNo_) + "P14" + szSetChar + std::to_string(maxVelocity));
      pCmdList->push_back(std::string(axisModuleNo_) + "P04" + szSetChar + std::to_string(minVelocity));
      break;
    case homeMove: //Set maximum velocity (P08) and minimum velocity (P10)
      pCmdList->push_back(std::string(axisModuleNo_) + "P08" + szSetChar + std::to_string(maxVelocity));
      pCmdList->push_back(std::string(axisModuleNo_) + "P10" + szSetChar + std::to_string(minVelocity));
      break;
    default:
      return phytronSuccess;
  }
  if (pCmdList != &asCommands) return phytronSuccess;
  return pC_->sendPhytronMultiCommand(asCommands, asCommands, false, iPollMethod == pollMethodSerial);
}

/** Sets acceleration parameters before the move is executed.
 * \param[in,out] pCmdList      list of commands to append to or NULL (execute directly)
 * \param[in]     acceleration  Acceleration to be used in the move
 * \param[in]     moveType      Type of movement determines which controller acceleration parameters is set
 */
phytronStatus phytronAxis::setAcceleration(std::vector<std::string>* pCmdList, double acceleration, int moveType)
{
  std::string sCommand;
  const char* szSetChar((pC_->iCtrlType_ == phytronController::TYPE_PHYMOTION) ? "=" : "S");
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
  sCommand = std::string(axisModuleNo_) + ((moveType == homeMove) ? "P09" : "P15") +
             szSetChar + std::to_string(acceleration);
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
  std::vector<std::string> asCommands;
  //printf("setting brake output for axis %s: wantmove=%d output=%.15g disable=%d engagetime=%.15g releasetime=%.15g released=%d\n",
  //       axisModuleNo_, bWantToMoveMotor, brakeOutput_, disableMotor_, brakeEngageTime_, brakeReleaseTime_, brakeReleased_);
  if (!pCmdList) pCmdList = &asCommands;
  brakeReleased_ = bWantToMoveMotor;
  if (bWantToMoveMotor) {
    // activate motor output
    if (!(disableMotor_ & 2)) {
      pCmdList->push_back(std::string(axisModuleNo_) + "MA");
      disableMotor_ |= 2;
    }
    if (fabs(brakeOutput_) > 0.0) {
      // release brake
      if (pC_->iCtrlType_ == phytronController::TYPE_PHYMOTION)
        sprintf(pC_->outString_, "A%.1f%c", fabs(brakeOutput_), brakeOutput_ > 0.0 ? 'S' : 'R');
      else
        sprintf(pC_->outString_, "A%d%c", static_cast<int>(fabs(10. * brakeOutput_ + 0.5)) % 10, brakeOutput_ > 0.0 ? 'S' : 'R');
      pCmdList->push_back(const_cast<const char*>(pC_->outString_));
    }

    if (pCmdList == &asCommands || brakeReleaseTime_ > 0.0) {
      pC_->sendPhytronMultiCommand(*pCmdList, asCommands, false, false);
      pCmdList->clear();
    }

    if (brakeReleaseTime_ > 0.0)
      epicsThreadSleep(brakeReleaseTime_);
    return phytronSuccess;
  }

  if (fabs(brakeOutput_) > 0.0) {
    // engage brake
    if (pC_->iCtrlType_ == phytronController::TYPE_PHYMOTION)
      sprintf(pC_->outString_, "A%.1f%c", fabs(brakeOutput_), brakeOutput_ > 0.0 ? 'R' : 'S');
    else
      sprintf(pC_->outString_, "A%d%c", static_cast<int>(fabs(10. * brakeOutput_ + 0.5)) % 10, brakeOutput_ > 0.0 ? 'R' : 'S');
    pCmdList->push_back(const_cast<const char*>(pC_->outString_));
    if (pCmdList == &asCommands || brakeEngageTime_ > 0.0) {
      pC_->sendPhytronMultiCommand(*pCmdList, asCommands, false, false);
      pCmdList->clear();
    }

    if (brakeEngageTime_ > 0.0)
      epicsThreadSleep(brakeEngageTime_);
  }
  if (disableMotor_ & 1) {
    // deactivate motor output
    pCmdList->push_back(std::string(axisModuleNo_) + "MD");
    disableMotor_ &= ~2;
    if (pCmdList == &asCommands) {
      pC_->sendPhytronMultiCommand(*pCmdList, asCommands, false, false);
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

  phyStatus = clearAxisError(&asCommands);
  CHECK_AXIS("move", "Clearing motor errors", this, return pC_->phyToAsyn(phyStatus));

  //NOTE: Check if velocity is different, before setting it.
  phyStatus = setVelocity(&asCommands, minVelocity, maxVelocity, stdMove);
  CHECK_AXIS("move", "Setting the velocity", this, return pC_->phyToAsyn(phyStatus));

  //NOTE: Check if velocity is different, before setting it.
  phyStatus = setAcceleration(&asCommands, acceleration, stdMove);
  CHECK_AXIS("move", "Setting the acceleration", this, return pC_->phyToAsyn(phyStatus));

  setBrakeOutput(&asCommands, 1);
  CHECK_AXIS("move", "Setting the brake", this, return pC_->phyToAsyn(phyStatus));

  if (relative) {
    sprintf(pC_->outString_, "%s%c%d", axisModuleNo_, position>0 ? '+':'-', abs(NINT(position)));
  } else {
    sprintf(pC_->outString_, "%sA%d", axisModuleNo_, NINT(position));
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
  int homingType(-1);
  enum pollMethod iPollMethod(iPollMethod_);
  const char* szSetChar((pC_->iCtrlType_ == phytronController::TYPE_PHYMOTION) ? "=" : "S");
  if (iPollMethod < 0) iPollMethod = pC_->iDefaultPollMethod_;

  pC_->getIntegerParam(axisNo_, pC_->homingProcedure_, &homingType);

  phyStatus = clearAxisError(&asCommands);
  CHECK_AXIS("move", "Clearing motor errors", this, return pC_->phyToAsyn(phyStatus));

  phyStatus =  setVelocity(&asCommands, minVelocity, maxVelocity, homeMove);
  CHECK_AXIS("home", "Setting the velocity", this, return pC_->phyToAsyn(phyStatus));

  phyStatus =  setAcceleration(&asCommands, acceleration, homeMove);
  CHECK_AXIS("home", "Setting the acceleration", this, return pC_->phyToAsyn(phyStatus));

  setBrakeOutput(&asCommands, 1);
  CHECK_AXIS("home", "Setting the brake", this, return pC_->phyToAsyn(phyStatus));

  if (axisNo_>= 10 && (axisNo_ / 10) <= ARRAY_SIZE(pC_->fake_homed_cache_))
  {
    int iIndex((axisNo_ / 10) - 1);
    if ((pC_->fake_homed_cache_[iIndex] >> (axisNo_ % 10)) & 1)
    {
      pC_->fake_homed_cache_[iIndex] &= ~(1 << (axisNo_ % 10));
      if (pC_->fake_homed_enable_)
      {
        if (!iIndex)
          pC_->fake_homed_cache_[0] |= 1;
        asCommands.push_back(std::string("R") + std::to_string(1001 + iIndex) + szSetChar +
                             std::to_string(pC_->fake_homed_cache_[iIndex]));
      }
    }
  }

  if (homingType == limit)
    homeState_ = forwards ? 3 : 2;
  if(forwards){
    if(homingType == limit) sprintf(pC_->outString_, "%sR+", axisModuleNo_);
    else if(homingType == center) sprintf(pC_->outString_, "%sR+C", axisModuleNo_);
    else if(homingType == encoder) sprintf(pC_->outString_, "%sR+I", axisModuleNo_);
    else if(homingType == limitEncoder) sprintf(pC_->outString_, "%sR+^I", axisModuleNo_);
    else if(homingType == centerEncoder) sprintf(pC_->outString_, "%sR+C^I", axisModuleNo_);
    //Homing procedures for rotational movements (no hardware limit switches)
    else if(homingType == referenceCenter) sprintf(pC_->outString_, "%sRC+", axisModuleNo_);
    else if(homingType == referenceCenterEncoder) sprintf(pC_->outString_, "%sRC+^I", axisModuleNo_);
    else return asynError;
  } else {
    if(homingType == limit) sprintf(pC_->outString_, "%sR-", axisModuleNo_);
    else if(homingType == center) sprintf(pC_->outString_, "%sR-C", axisModuleNo_);
    else if(homingType == encoder) sprintf(pC_->outString_, "%sR-I", axisModuleNo_);
    else if(homingType == limitEncoder) sprintf(pC_->outString_, "%sR-^I", axisModuleNo_);
    else if(homingType == centerEncoder) sprintf(pC_->outString_, "%sR-C^I", axisModuleNo_);
    //Homing procedures for rotational movements (no hardware limit switches)
    else if(homingType == referenceCenter) sprintf(pC_->outString_, "%sRC-", axisModuleNo_);
    else if(homingType == referenceCenterEncoder) sprintf(pC_->outString_, "%sRC-^I", axisModuleNo_);
    else return asynError;
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

  phyStatus = clearAxisError(&asCommands);
  CHECK_AXIS("move", "Clearing motor errors", this, return pC_->phyToAsyn(phyStatus));

  phyStatus = setVelocity(&asCommands, minVelocity, maxVelocity, stdMove);
  CHECK_AXIS("moveVelocity", "Setting the velocity", this, );

  phyStatus = setAcceleration(&asCommands, acceleration, stdMove);
  CHECK_AXIS("moveVelocity", "Setting the acceleration", this, );

  setBrakeOutput(&asCommands, 1);

  asCommands.push_back(std::string(axisModuleNo_) + (maxVelocity < 0. ? "L-" : "L+"));

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

  asCommands.push_back(std::string(axisModuleNo_) + "S");
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
  const char* szSetChar((pC_->iCtrlType_ == phytronController::TYPE_PHYMOTION) ? "=" : "S");

  phyStatus = pC_->sendPhytronCommand(std::string(axisModuleNo_) + "P22R", sResponse);
  if (phyStatus == phytronSuccess)
    if (epicsParseDouble(sResponse.c_str(), &encoderPosition, NULL) != 0)
      phyStatus = phytronInvalidReturn;
  CHECK_AXIS("setEncoderRatio", "Reading encoder position", this, return pC_->phyToAsyn(phyStatus));

  phyStatus = pC_->sendPhytronCommand(std::string(axisModuleNo_) + "P39" + szSetChar
                                      + std::to_string(1. / ratio), sResponse);
  CHECK_AXIS("setEncoderRatio", "Setting ratio", this, return pC_->phyToAsyn(phyStatus));

  setDoubleParam(pC_->motorEncoderPosition_, encoderPosition * ratio);
  callParamCallbacks();
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
  std::vector<std::string> asCommands;
  const char* szSetChar((pC_->iCtrlType_ == phytronController::TYPE_PHYMOTION) ? "=" : "S");

  asCommands.push_back(std::string(axisModuleNo_) + "P20" + szSetChar + std::to_string(position));
  if (axisNo_>= 10 && (axisNo_ / 10) <= ARRAY_SIZE(pC_->fake_homed_cache_))
  {
    int iIndex((axisNo_ / 10) - 1);
    if (!((pC_->fake_homed_cache_[iIndex] >> (axisNo_ % 10)) & 1))
    {
      pC_->fake_homed_cache_[iIndex] |= 1 << (axisNo_ % 10);
      if (pC_->fake_homed_enable_)
      {
        if (!iIndex)
          pC_->fake_homed_cache_[0] |= 1;
        asCommands.push_back(std::string("R") + std::to_string(1001 + iIndex) + szSetChar
                             + std::to_string(pC_->fake_homed_cache_[iIndex]));
      }
    }
  }
  phyStatus = pC_->sendPhytronMultiCommand(asCommands, asCommands, false, iPollMethod_ == pollMethodSerial);
  CHECK_AXIS("setPosition", "Setting motor position", this, return pC_->phyToAsyn(phyStatus));

  return asynSuccess;
}

/** Polls the axis depending on configuration (asynSetOption "pollMethod").
 * pollMethodDefault: apply controller setting for this axis (see below)
 * pollMethodSerial (old or default):
 * - phyMOTION
 *   4 single command request and replies: motor position, encoder position,
 *   moving status, axis status (limit switches, home switches, ...),
 *   takes about 40ms per axis
 * - MCC
 *   4+ single command request and replies: motor position, encoder position,
 *   moving status, error status and plus one limit switches in controller poll
 * pollMethodAxisParallel:
 * pollMethodControllerParallel:
 * - phyMOTION only
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
  std::vector<std::string> asCommands, asResponses;
  phytronStatus phyStatus(phytronSuccess);
  if (iPollMethod < 0) iPollMethod = pC_->iDefaultPollMethod_;
  switch (iPollMethod) {
    case pollMethodSerial:
    case pollMethodAxisParallel:
      appendRequestString(asCommands); // get commands
      // communicate
      phyStatus = pC_->sendPhytronMultiCommand(asCommands, asResponses, false, iPollMethod == pollMethodSerial);
      if (phyStatus == phytronSuccess) // on success: parse answers
      {
        if (!parseAnswer(asResponses) || !asResponses.empty())
          phyStatus = phytronInvalidReturn;
      }
      else
        setIntegerParam(pC_->motorStatusProblem_, 1); // set problem bit
      CHECK_AXIS("poll", "Reading axis", this, setIntegerParam(pC_->motorStatusProblem_, 1); \
        callParamCallbacks());
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
  switch (pC_->iCtrlType_)
  {
    case phytronController::TYPE_PHYMOTION:
      if (iPollMethod == pollMethodSerial) {
        // old default method, which uses 4 requests
        asCommands.push_back(std::string(pszAxis) + "P20R");
        asCommands.push_back(std::string(pszAxis) + "P22R");
        asCommands.push_back(std::string(pszAxis) + "==H");
        asCommands.push_back(std::string("SE") + &pszAxis[1]);
      } else {
        // new method: fix race condition and substitutes "==H" with result of "SE"-bit 0
        asCommands.push_back(std::string("SE") + &pszAxis[1]);
        asCommands.push_back(std::string(pszAxis) + "P20R");
        asCommands.push_back(std::string(pszAxis) + "P22R");
      }
      break;
    case phytronController::TYPE_MCC:
      asCommands.push_back(std::string(pszAxis) + "=H"); // idle
      asCommands.push_back(std::string(pszAxis) + "=E"); // power-stage error
      asCommands.push_back(std::string(pszAxis) + "P20R");
      asCommands.push_back(std::string(pszAxis) + "P22R");
      break;
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
  int iAxisStatus(0), iMotIdx(-1), iEncIdx(-1), iStatIdx(-1), iIdleIdx(-1), iHighLimit(0), iLowLimit(0);
  size_t iRemoveCount(static_cast<size_t>(-1));
  bool bResult(false), bMoving;
  double dRatio;
  epicsInt32 iOldDone(0);
  switch (pC_->iCtrlType_)
  {
    case phytronController::TYPE_PHYMOTION:
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
      if (asValues.size() < iRemoveCount)
        goto finish;
      if (iPollMethod == pollMethodControllerParallel) {
        for (size_t i = 0; i < iRemoveCount; ++i) {
          if (asValues[i].substr(0, 1) != "\x06") // no ACK found here
            goto finish;
          asValues[i].erase(0, 1);
        }
      }
      break;
    case phytronController::TYPE_MCC:
    {
      int iAxis(-1);
      if (asValues.size() < 2)
        goto finish;
      iRemoveCount = 4;
      switch (axisModuleNo_[0])
      {
        case 'X': case 'x': case '1': iAxis = 0; break;
        case 'Y': case 'y': case '2': iAxis = 1; break;
        case 'Z': case 'z': case '3': iAxis = 2; break;
        case 'W': case 'w': case '4': iAxis = 3; break;
        case 'A': case 'a': case '5': iAxis = 4; break;
        case 'B': case 'b': case '6': iAxis = 5; break;
        case 'C': case 'c': case '7': iAxis = 6; break;
        case 'D': case 'd': case '8': iAxis = 7; break;
        default: goto finish;
      }
      if (pC_->sLastSUI_.size() > static_cast<size_t>(iAxis + 2) && pC_->sLastSUI_[0] == 'I' && pC_->sLastSUI_[1] == '=')
      {
        switch (pC_->sLastSUI_[2 + iAxis])
        {
          case '0': iAxisStatus = 0x00; break; // no limit switch
          case '2': iAxisStatus = 0x30; break; // both limit switches
          case '+': iAxisStatus = 0x10; break; // positive limit switch
          case '-': iAxisStatus = 0x20; break; // negative limit switch
          default: goto finish;
        }
      }
      else
        goto finish;
      if (asValues[1].substr(0, 1) == "E") // X=E power-stage error
        iAxisStatus |= 0x2000; // power stage error
      iIdleIdx = 0; // "X=H"
      iStatIdx = -1;
      iMotIdx  = 2; // XP20R
      iEncIdx  = 3; // XP22R
      break;
    }
    default:
      goto finish;
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
  if (iStatIdx >= 0)
    iAxisStatus = atoi(asValues[iStatIdx].c_str());

  // idle/moving status
  pC_->getIntegerParam(axisNo_, pC_->motorStatusDone_, &iOldDone);
  if (iIdleIdx >= 0)
  {
    bMoving = (asValues[iIdleIdx].substr(0, 1) != "E");
    if (iStatIdx < 0 && bMoving)
      iAxisStatus |= 0x01;
  }
  else
    bMoving = (iAxisStatus & 1) != 0;
  setIntegerParam(pC_->motorStatusDone_, bMoving ? 0 : 1);
  if (!iOldDone && !bMoving) // axis stopped
    setBrakeOutput(NULL, 0);

  iHighLimit = (iAxisStatus & 0x10) ? 1 : 0;
  iLowLimit  = (iAxisStatus & 0x20) ? 1 : 0;
  if (homeState_ >> 1) {
    // workaround for home on limit switch
    int iHomingType(-1);
    bResult = !(iAxisStatus & 0xE800); // axis internal/power-stage/SFI/ENDAT error
    if (homeState_ & 1)
      iHighLimit = 0;
    else
      iLowLimit = 0;
    pC_->getIntegerParam(axisNo_, pC_->homingProcedure_, &iHomingType);
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
        {
          // end-of-reference: in case of triggered limit switches, Phytron
          // sets "limit switch error", which should be cleared, if no
          // other problem was detected [manual SEm.n]
          int iMask((homeState_ & 1) ? 0xFA29 : 0xFA19);
          if (pC_->iCtrlType_ == phytronController::TYPE_PHYMOTION && (iAxisStatus & iMask) == 0x1208)
            bResult = (pC_->sendPhytronCommand(std::string("SEC") + &axisModuleNo_[1]) == phytronSuccess);
          homeState_ = 0;
        }
        break;
      default:
        homeState_ = 0;
        break;
    }
  }
  else
    bResult = !(iAxisStatus & 0xE800); // axis internal/power-stage/SFI/ENDAT error
  if ((iAxisStatus & 0x1000) != 0 && statusResetTime_ < 0.) // axis limit-switch error
    statusResetTime_ = -statusResetTime_;
  setIntegerParam(pC_->motorStatusHighLimit_, iHighLimit);
  setIntegerParam(pC_->motorStatusLowLimit_,  iLowLimit);
  setIntegerParam(pC_->motorStatusAtHome_, (iAxisStatus & 0x40)/0x40);
  setIntegerParam(pC_->motorStatusHomed_, 0);
  if (pC_->fake_homed_enable_ && axisNo_ >= 10 &&
      (axisNo_ / 10) <= ARRAY_SIZE(pC_->fake_homed_cache_) &&
      ((pC_->fake_homed_cache_[(axisNo_ / 10) - 1] >> (axisNo_ % 10)) & 1))
    setIntegerParam(pC_->motorStatusHomed_, 1); // after setPosition: set HOMED bit
  else
    setIntegerParam(pC_->motorStatusHomed_, (iAxisStatus & 0x08)/0x08);
  setIntegerParam(pC_->motorStatusHome_,   (iAxisStatus & 0x08)/0x08);
  setIntegerParam(pC_->motorStatusSlip_,   (iAxisStatus & 0x4000)/0x4000);
  setIntegerParam(pC_->axisStatus_, iAxisStatus); // Update the axis status record ($(P)$(M)_STATUS)

finish:
  setIntegerParam(pC_->motorStatusProblem_, bResult ? 0 : 1);
  callParamCallbacks();
  asValues.erase(asValues.begin(), asValues.begin() + iRemoveCount);
  return bResult;
}
