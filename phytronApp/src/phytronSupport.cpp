/*
SPDX-License-Identifier: EPICS
FILENAME... phytronSupport.cpp
USAGE...    IOCSH Support for Phytron controller.

Lutz Rossa, Helmholtz-Zentrum Berlin fuer Materialien und Energy GmbH, 2025

*/
#include <iocsh.h>
#include <epicsExport.h>
#include "phytronAxisMotor.h"
#include "phytronIOctrl.h"

//static size of fixed array
#define ARRAY_SIZE(x) ((int)(sizeof(x) / sizeof((x)[0])))

//******************************************************************************
//                   IOCSH SPECIFIC IMPLEMENTATION
//******************************************************************************

/** Parameters for iocsh Phytron axis registration*/
static const iocshArg phytronCreateAxisArg0 = {"Controller Name", iocshArgString};
static const iocshArg phytronCreateAxisArg1 = {"Module index", iocshArgInt};
static const iocshArg phytronCreateAxisArg2 = {"Axis index", iocshArgInt};
static const iocshArg* const phytronCreateAxisArgs[] = {&phytronCreateAxisArg0,
                                                        &phytronCreateAxisArg1,
                                                        &phytronCreateAxisArg2};

/** Parameters for iocsh Phytron phyMOTION controller registration */
static const iocshArg phytronCreatePhymotionArg0 = {"Controller name", iocshArgString};
static const iocshArg phytronCreatePhymotionArg1 = {"Phytron communication port name", iocshArgString};
static const iocshArg phytronCreatePhymotionArg2 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg phytronCreatePhymotionArg3 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg phytronCreatePhymotionArg4 = {"Timeout (ms)", iocshArgDouble};
static const iocshArg phytronCreatePhymotionArg5 = {"Do not restart controller with IOC", iocshArgInt};
static const iocshArg * const phytronCreatePhymotionArgs[] = {&phytronCreatePhymotionArg0,
                                                              &phytronCreatePhymotionArg1,
                                                              &phytronCreatePhymotionArg2,
                                                              &phytronCreatePhymotionArg3,
                                                              &phytronCreatePhymotionArg4,
                                                              &phytronCreatePhymotionArg5};

/** Parameters for iocsh Phytron MCC controller registration */
static const iocshArg phytronCreateMCCArg0 = {"Controller name", iocshArgString};
static const iocshArg phytronCreateMCCArg1 = {"Phytron communication port name", iocshArgString};
static const iocshArg phytronCreateMCCArg2 = {"Controller address (0..15)", iocshArgInt};
static const iocshArg phytronCreateMCCArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg phytronCreateMCCArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg phytronCreateMCCArg5 = {"Timeout (ms)", iocshArgDouble};
static const iocshArg phytronCreateMCCArg6 = {"Do not restart controller with IOC", iocshArgInt};
static const iocshArg * const phytronCreateMCCArgs[] = {&phytronCreateMCCArg0,
                                                        &phytronCreateMCCArg1,
                                                        &phytronCreateMCCArg2,
                                                        &phytronCreateMCCArg3,
                                                        &phytronCreateMCCArg4,
                                                        &phytronCreateMCCArg5,
                                                        &phytronCreateMCCArg6};


/** Parameters for iocsh Phytron brake(s) output registration */
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

/** Parameters for iocsh Phytron I/O controller registration */
static const iocshArg phytronCreateIOArg0 = {"IO Controller Name", iocshArgString};
static const iocshArg phytronCreateIOArg1 = {"Phytron Controller Name", iocshArgString};
static const iocshArg phytronCreateIOArg2 = {"card number (1..)", iocshArgInt};
static const iocshArg phytronCreateIOArg3 = {"channel number (1..)", iocshArgInt};
static const iocshArg phytronCreateIOArg4 = {"initial commands string", iocshArgString};
static const iocshArg* const phytronCreateIOArgs[] = {&phytronCreateIOArg0,
                                                      &phytronCreateIOArg1,
                                                      &phytronCreateIOArg2,
                                                      &phytronCreateIOArg3,
                                                      &phytronCreateIOArg4};

/** IOCSH functions */
static const iocshFuncDef phytronCreateAxisDef = {"phytronCreateAxis", ARRAY_SIZE(phytronCreateAxisArgs), phytronCreateAxisArgs
#if defined(EPICS_VERSION)
#if EPICS_VERSION > 7 || (EPICS_VERSION == 7 && (EPICS_REVISION > 0 || EPICS_MODIFICATION >= 3))
    , "Create an axis instance for one motor of any previously created Phytron controller.\n"
#endif
#endif
};
static const iocshFuncDef phytronCreatePhymotionDef = {"phytronCreateController", ARRAY_SIZE(phytronCreatePhymotionArgs), phytronCreatePhymotionArgs
#if defined(EPICS_VERSION)
#if EPICS_VERSION > 7 || (EPICS_VERSION == 7 && (EPICS_REVISION > 0 || EPICS_MODIFICATION >= 3))
    , "Create a Phytron phyMOTION controller instance.\n"
#endif
#endif
};
static const iocshFuncDef phytronCreateMCCDef = {"phytronCreateMCC", ARRAY_SIZE(phytronCreateMCCArgs), phytronCreateMCCArgs
#if defined(EPICS_VERSION)
#if EPICS_VERSION > 7 || (EPICS_VERSION == 7 && (EPICS_REVISION > 0 || EPICS_MODIFICATION >= 3))
    , "Create a Phytron MCC-1 or MCC-2 controller instance.\n"
#endif
#endif
};
static const iocshFuncDef phytronBrakeOutputDef = {"phytronBrakeOutput", ARRAY_SIZE(phytronBrakeOutputArgs), phytronBrakeOutputArgs
#if defined(EPICS_VERSION)
#if EPICS_VERSION > 7 || (EPICS_VERSION == 7 && (EPICS_REVISION > 0 || EPICS_MODIFICATION >= 3))
    , "Configure brake support for a Phytron motor axis.\n"
#endif
#endif
};
static const iocshFuncDef phytronCreateDIODef = {"phytronCreateDigital", ARRAY_SIZE(phytronCreateIOArgs), phytronCreateIOArgs
#if defined(EPICS_VERSION)
#if EPICS_VERSION > 7 || (EPICS_VERSION == 7 && (EPICS_REVISION > 0 || EPICS_MODIFICATION >= 3))
    , "Create a Phytron digital I/O instance for any previously created Phytron controller.\n"
#endif
#endif
};
static const iocshFuncDef phytronCreateAIODef = {"phytronCreateAnalog", ARRAY_SIZE(phytronCreateIOArgs), phytronCreateIOArgs
#if defined(EPICS_VERSION)
#if EPICS_VERSION > 7 || (EPICS_VERSION == 7 && (EPICS_REVISION > 0 || EPICS_MODIFICATION >= 3))
    , "Create a Phytron analog I/O instance for any previously created Phytron controller.\n"
#endif
#endif
};

static void phytronCreatePhymotionCallFunc(const iocshArgBuf *args)
{
  phytronController::phytronCreatePhymotion(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].dval,args[5].ival);
}

static void phytronCreateMCCCallFunc(const iocshArgBuf *args)
{
  phytronController::phytronCreateMCC(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].dval, args[6].ival);
}

static void phytronCreateAxisCallFunc(const iocshArgBuf *args)
{
  phytronAxis::phytronCreateAxis(args[0].sval, args[1].ival, args[2].ival);
}

static void phytronBrakeOutputCallFunc(const iocshArgBuf *args)
{
  phytronAxis::phytronBrakeOutput(args[0].sval, args[1].dval, args[2].dval, args[3].ival, args[4].dval, args[5].dval);
}

static void phytronCreateDigitalIOcontrollerCallFunc(const iocshArgBuf *args)
{
  phytronIO::createIOcontroller(args[0].sval, args[1].sval, false, args[2].ival, args[3].ival, args[4].sval);
}

static void phytronCreateAnalogIOcontrollerCallFunc(const iocshArgBuf *args)
{
  phytronIO::createIOcontroller(args[0].sval, args[1].sval, true, args[2].ival, args[3].ival, args[4].sval);
}

static void phytronRegister(void)
{
  iocshRegister(&phytronCreatePhymotionDef, phytronCreatePhymotionCallFunc);
  iocshRegister(&phytronCreateMCCDef, phytronCreateMCCCallFunc);
  iocshRegister(&phytronCreateAxisDef, phytronCreateAxisCallFunc);
  iocshRegister(&phytronBrakeOutputDef, phytronBrakeOutputCallFunc);
  iocshRegister(&phytronCreateDIODef, phytronCreateDigitalIOcontrollerCallFunc);
  iocshRegister(&phytronCreateAIODef, phytronCreateAnalogIOcontrollerCallFunc);
}

extern "C" {
epicsExportRegistrar(phytronRegister);
}
