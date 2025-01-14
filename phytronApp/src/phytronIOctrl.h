/*
SPDX-License-Identifier: EPICS
FILENAME... phytronIOctrl.h
USAGE...    I/O support for Phytron controller.

Bernhard Kuner, Helmholtz-Zentrum Berlin fuer Materialien und Energy GmbH, 2022-2023
Lutz Rossa, Helmholtz-Zentrum Berlin fuer Materialien und Energy GmbH, 2024-2025

*/
#include <asynPortDriver.h>
#include <map>

struct phytronIOparam;
class phytronController;
class phytronIO : public asynPortDriver
{
public:
  enum IOTYPE { TYPE_GUESS_D, TYPE_GUESS_A, TYPE_DI, TYPE_DO, TYPE_DIO, TYPE_AI, TYPE_AO, TYPE_AIO };

  // common asyn functions
  phytronIO(const char* szIOPortName, phytronController* pCtrl, IOTYPE iType, epicsUInt8 byCardNr, epicsUInt8 byChannel, epicsInt32 iIn, epicsInt32 iOut);
  asynStatus readInt32(asynUser* pasynUser, epicsInt32* piValue);
  asynStatus writeInt32(asynUser* pasynUser, epicsInt32 iValue);
  asynStatus writeOctet(asynUser* pasynUser, const char* szValue, size_t maxChars, size_t* pnActual);
  void report(FILE* pFile, int iLevel);

  // I/O controller functions
  static int createIOcontroller(const char* szAsynPortName, const char* szPhytronPortName, bool bAnalog, epicsUInt8 byCardNr, epicsUInt8 byChannel, const char* szInitCommands);
  bool pollIO(std::vector<std::string>& asCmdReplyList, bool bIsRequest);
  static bool pollIO(std::vector<std::string>& asCmdReplyList, bool bIsRequest, phytronController* pCtrl, IOTYPE& iType, epicsUInt8 byCardNr, epicsUInt8 byChannel, epicsInt32& iIn, epicsInt32& iOut);

protected:
  std::map<int, struct phytronIOparam*> m_mapParameters; ///< mapping of asyn reasons to parameter
  phytronController* pCtrl_;         ///< controller instance
  IOTYPE             iType_;         ///< card type
  epicsUInt8         byCardNr_;      ///< I/O card number
  epicsUInt8         byChannel_;     ///< I/O channel number
  epicsInt32         iIn_;           ///< last polled input value
  epicsInt32         iOut_;          ///< last polled output value
  int                iInReason_;     ///< index of IN parameter
  int                iOutReason_;    ///< index of OUT parameter
  int                iDirReason_;    ///< index of DIR parameter
  int                iStatusReason_; ///< index of STATUS parameter
  int                iReplyReason_;  ///< index of REPLY parameter

friend class phytronController;
};
