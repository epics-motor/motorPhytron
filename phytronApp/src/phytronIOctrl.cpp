/*
SPDX-License-Identifier: EPICS
FILENAME... phytronIOctrl.cpp
USAGE...    I/O support for Phytron controller.

Bernhard Kuner, Helmholtz-Zentrum Berlin fuer Materialien und Energy GmbH, 2022-2023
Lutz Rossa, Helmholtz-Zentrum Berlin fuer Materialien und Energy GmbH, 2025
*/
#include <epicsStdlib.h>
#include <iocsh.h>
#include <asynPortClient.h>
#include "phytronIOctrl.h"
#include "phytronAxisMotor.h"

//******************************************************************************
//                   I/O CONTROLLER IMPLEMENTATION
//******************************************************************************

//static size of fixed array
#define ARRAY_SIZE(x) ((int)(sizeof(x) / sizeof((x)[0])))

struct phytronIOparam
{
  bool          bWriteable;  ///< read only parameter cannot be written
  bool          bMCC1only;   ///< parameter for MCC-1 only
  asynParamType iAsynType;   ///< asyn type of parameter type
  const char*   szAsynName;  ///< asyn parameter name
};

static struct phytronIOparam g_aParameters[] =
{
  { false, false, asynParamInt32, "IN"  },
  { true,  false, asynParamInt32, "OUT" },
  { true,  true,  asynParamInt32, "DIR" },
};

/**
 * \brief phytronIO::phytronIO
 * \param[in] szIOPortName  The name of the asyn port that will be created for this driver.
 * \param[in] pCtrl         Pointer to existing Phytron controller.
 * \param[in] iType         Card type.
 * \param[in] byCardNr      I/O card number (starting with 1).
 * \param[in] byChannel     channel number (starting with 1).
 * \param[in] byDIn         Initial digital inputs state.
 * \param[in] byDOut        Initial digital outputs state.
 */
phytronIO::phytronIO(const char* szIOPortName, phytronController* pCtrl, phytronIO::IOTYPE iType, epicsUInt8 byCardNr, epicsUInt8 byChannel, epicsInt32 iIn, epicsInt32 iOut)
  : asynPortDriver(szIOPortName,
                   1, // maximum address
#if ASYN_VERSION < 4 || (ASYN_VERSION == 4 && ASYN_REVISION < 32)
                   ARRAY_SIZE(g_aParameters),
#endif
                   asynInt32Mask | asynDrvUserMask, // additional interfaces
                   asynInt32Mask, // additional callback interfaces
                   ASYN_CANBLOCK, // asynFlags
                   1, // autoConnect
                   0, // default priority
                   0) // default stackSize
  , pCtrl_(pCtrl)
  , iType_(iType)
  , byCardNr_(byCardNr)
  , byChannel_(byChannel)
  , iIn_(iIn)
  , iOut_(iOut)
  , iInReason_(-1)
  , iOutReason_(-1)
  , iDirReason_(-1)
{
  for (size_t i = 0; i < ARRAY_SIZE(g_aParameters); ++i)
  {
    struct phytronIOparam* p(&g_aParameters[i]);
    int iReason(-1);
    if (p->bMCC1only)
    {
      if (pCtrl->iCtrlType_ != phytronController::TYPE_MCC)
        continue;
      if (epicsStrnCaseCmp(pCtrl->sCtrlType_.c_str(), "MCC-1", 5) != 0)
        continue;
    }
    if (createParam(p->szAsynName, p->iAsynType, &iReason) == asynSuccess && iReason >= 0)
    {
      m_mapParameters[iReason] = p;
      if (strcmp(p->szAsynName, "IN") == 0)
      {
        iInReason_ = iReason;
        setIntegerParam(iReason, iIn);
      }
      else if (strcmp(p->szAsynName, "OUT") == 0)
      {
        iOutReason_ = iReason;
        setIntegerParam(iReason, iOut);
      }
    }
  }
  pCtrl->lock();
  pCtrl->IOs_.push_back(this);
  pCtrl->unlock();
}

/**
 * \brief create a new I/O controller phytronController::createIOcontroller
 * \param[in] szPhytronPortName  The name of the asyn port that will be created for this driver.
 * \param[in] szAsynPortName     The name of the asyn port for the existing Phytron controller.
 * \param[in] bAnalog            Card type.
 * \param[in] byCardNr           I/O card number (starting with 1).
 * \param[in] byChannel          channel number (starting with 1).
 * \param[in] szInitCommands     Initial commands to send or empty.
 * \return asynSuccess or asynError
 */
int phytronIO::createIOcontroller(const char* szAsynPortName, const char* szPhytronPortName, bool bAnalog, epicsUInt8 byCardNr, epicsUInt8 byChannel, const char* szInitCommands)
{
  epicsUInt32 i;
  phytronIO::IOTYPE iType(bAnalog ? TYPE_GUESS_A : TYPE_GUESS_D);
  if (!szAsynPortName || !*szAsynPortName || !szPhytronPortName || !*szPhytronPortName)
    goto failed;
  for (i = 0; i < phytronController::controllers_.size(); ++i)
  {
    phytronController* pC(phytronController::controllers_[i]);
    if (!pC || strcmp(pC->controllerName_, szPhytronPortName) != 0)
      continue;
    pC->lock();
    std::vector<std::string> asCommands, asReplies;
    epicsInt32 iIn(0), iOut(0);
    bool bOk(false);
    pollIO(asCommands, true, pC, iType, byCardNr, byChannel, iIn, iOut);
    if (szInitCommands && *szInitCommands)
      asCommands.push_back(szInitCommands);
    if (pC->sendPhytronMultiCommand(asCommands, asReplies, true, pC->iCtrlType_ != phytronController::TYPE_PHYMOTION) == phytronSuccess)
      bOk = pollIO(asReplies, false, pC, iType, byCardNr, byChannel, iIn, iOut);
    pC->unlock();
    if (bOk)
    {
      new phytronIO(szAsynPortName, pC, iType, byCardNr,byChannel, iIn, iOut);
      return asynSuccess;
    }
  }

failed:
  printf("ERROR: invalid call to phytronCreateIOcontroller\n");
  return asynError;
}

/**
 * \brief Called when asyn clients call pasynInt32->read().
 *        If this is parameter registered here, it will read the hardware. In other cases,
 *        this will call the base class, which simply returns the stored value.
 * \param[in]  pasynUser  pasynUser structure that encodes the reason and address.
 * \param[out] piValue    address of the value to read.
 * \return asyn result code
 */
asynStatus phytronIO::readInt32(asynUser* pasynUser, epicsInt32* piValue)
{
  asynStatus iResult(asynError);
  struct phytronIOparam* pParam(nullptr);
  if (pasynUser->reason >= 0 && m_mapParameters.count(pasynUser->reason))
    pParam = m_mapParameters[pasynUser->reason];
  if (!pParam) // default handler for other asyn parameters
    iResult = asynPortDriver::readInt32(pasynUser, piValue);
  else if (pParam->iAsynType == asynParamInt32)
  {
    if (pParam->bWriteable)
      *piValue = iOut_;
    else
      *piValue = iIn_;
    iResult = asynSuccess;
  }
  else
    iResult = asynError;
  return iResult;
}

asynStatus phytronIO::writeInt32(asynUser* pasynUser, epicsInt32 iValue)
{
  asynStatus iResult(asynSuccess);
  struct phytronIOparam* pParam(nullptr);
  if (pasynUser->reason >= 0 && m_mapParameters.count(pasynUser->reason))
    pParam = m_mapParameters[pasynUser->reason];
  if (!pParam) // default handler for other asyn parameters
    goto handlewrite;
  if (!pParam->bWriteable || pParam->iAsynType != asynParamInt32)
    return asynError;

  char szCmd[32];
  szCmd[0] = '\0';
  if (pCtrl_->iCtrlType_ == phytronController::TYPE_MCC)
  {
    switch (iType_)
    {
      case TYPE_DO:
      case TYPE_DIO:
        snprintf(szCmd, ARRAY_SIZE(szCmd), "%s%uS%d%d%d%d%d%d%d%d",
                 (pasynUser->reason == iDirReason_) ? "EAS" : "AG",
                 byChannel_,
                 (iValue &   1) ? 1 : 0,
                 (iValue &   2) ? 1 : 0,
                 (iValue &   4) ? 1 : 0,
                 (iValue &   8) ? 1 : 0,
                 (iValue &  16) ? 1 : 0,
                 (iValue &  32) ? 1 : 0,
                 (iValue &  64) ? 1 : 0,
                 (iValue & 128) ? 1 : 0);
        break;
      default: return asynError;
    }
  }
  else
  {
    switch (iType_)
    {
      case TYPE_DO:
      case TYPE_DIO:
        snprintf(szCmd, ARRAY_SIZE(szCmd), "AG%u=%d", byCardNr_, iValue);
        break;
      case TYPE_AO:
      case TYPE_AIO:
        snprintf(szCmd, ARRAY_SIZE(szCmd), "DA%u.%u=%d", byCardNr_, byChannel_, iValue);
        break;
      default: return asynError;
    }
  }
  if (szCmd[0])
  {
    szCmd[sizeof(szCmd) - 1] = '\0';
    iResult = phytronController::phyToAsyn(pCtrl_->sendPhytronCommand(const_cast<const char*>(&szCmd[0])));
    if (iResult == asynSuccess)
      iOut_ = iValue;
  }
  else
    iResult = asynError;

handlewrite:
  if (iResult == asynSuccess)
    iResult = asynPortDriver::writeInt32(pasynUser, iValue);
  return iResult;
}

void phytronIO::report(FILE* pFile, int iLevel)
{
  epicsInt32 iMaxModule(-1);
  asynPortDriver::report(pFile, iLevel);
  const char* szType(nullptr);
  // all report levels: for this module only - controller type, cardnr, channel
  switch (iType_)
  {
    case TYPE_GUESS_D: szType = "guess digital"; break;
    case TYPE_GUESS_A: szType = "guess analog";  break;
    case TYPE_DI:      szType = "DIN";  break;
    case TYPE_DO:      szType = "DOUT"; break;
    case TYPE_DIO:     szType = "DIO";  break;
    case TYPE_AI:      szType = "AIN";  break;
    case TYPE_AO:      szType = "AOUT"; break;
    case TYPE_AIO:     szType = "AIO";  break;
    default:           szType = "?";    break;
  }
  fprintf(pFile, "%s %s: card=%u channel=%u in=%d out=%d\n",
          pCtrl_->sCtrlType_.c_str(), szType, byCardNr_, byChannel_, iIn_, iOut_);
  if (iLevel >= 1)
  {
    std::string sTmp;
    char* pUnits(nullptr);
    if (pCtrl_->sendPhytronCommand(std::string((pCtrl_->iCtrlType_ == phytronController::TYPE_MCC) ? "IAR" : "IV"), sTmp) == phytronSuccess)
      if (epicsParseInt32(sTmp.c_str(), &iMaxModule, 0, &pUnits) != 0)
        iMaxModule = -1;
    if (iMaxModule < 1)
      iMaxModule = (pCtrl_->iCtrlType_ == phytronController::TYPE_MCC) ? 8 : 21;
  }
  if (iLevel == 1)
  {
    // report level 1: detailled controller type (MCC-1, MCC-2, MCM-01/02/03/04, short module list)
    fprintf(pFile, "  controller: %s\n", pCtrl_->sCtrlType_.c_str());
    if (pCtrl_->iCtrlType_ != phytronController::TYPE_MCC)
    {
      pCtrl_->lock();
      for (int i = 1; i <= iMaxModule; ++i)
      {
        std::string sType;
        phytronStatus iResult(pCtrl_->sendPhytronCommand(std::string("IM") + std::to_string(i), sType));
        if (iResult == phytronSuccess)
          fprintf(pFile, "  module %d: %s\n", i, phytronController::escapeC(sType).c_str());
      }
      pCtrl_->unlock();
    }
  }
  else if (iLevel >= 2)
  {
    // report level 2: detailled version info of all
    pCtrl_->lock();
    if (pCtrl_->iCtrlType_ == phytronController::TYPE_MCC)
    {
      std::string sTmp;
      pCtrl_->sendPhytronCommand(std::string("IVR"), sTmp); // minilog version
      fprintf(pFile, "  controller: %s  %s\n", pCtrl_->sCtrlType_.c_str(), phytronController::escapeC(sTmp).c_str());
      for (int i = 1; i <= iMaxModule; ++i)
      {
        std::string sMod;
        if (pCtrl_->sendPhytronCommand(std::to_string(i) + std::string("VI"), sTmp) == phytronSuccess) // indexer version
        {
          if (!sMod.empty())
            sMod.append(", ");
          sMod.append(sTmp);
        }
        if (pCtrl_->sendPhytronCommand(std::to_string(i) + std::string("VL"), sTmp) == phytronSuccess) // loader version
        {
          if (!sMod.empty())
            sMod.append(", ");
          sMod.append(sTmp);
        }
        if (sMod.empty())
          sMod = "no version info";
        fprintf(pFile, "  axis %d: %s\n", i, phytronController::escapeC(sMod).c_str());
      }
    }
    else
    {
      for (int i = 0; i <= iMaxModule; ++i)
      {
        std::string sType;
        phytronStatus iResult(pCtrl_->sendPhytronCommand(std::string("IV") + std::to_string(i), sType));
        if (!i)
          fprintf(pFile, "  controller: %s\n", phytronController::escapeC(sType).c_str());
        else if (iResult == phytronSuccess)
          fprintf(pFile, "  module %d: %s\n", i, phytronController::escapeC(sType).c_str());
      }
    }
    pCtrl_->unlock();
  }
}

/**
 * \brief called by phytronController::poll cycle
 * \param[in,out] asCmdReplyList  bIsRequest=true: append command to command list, bIsRequest=false: parse and remove 1st reply from replies
 * \param[in]     bIsRequest      type of call
 */
bool phytronIO::pollIO(std::vector<std::string>& asCmdReplyList, bool bIsRequest)
{
  bool bOk(pollIO(asCmdReplyList, bIsRequest, pCtrl_, iType_, byCardNr_, byChannel_, iIn_, iOut_));
  if (bOk)
  {
    setIntegerParam(iInReason_, iIn_);
    setIntegerParam(iOutReason_, iOut_);
    callParamCallbacks();
  }
  return bOk;
}

/**
 * \brief called by phytronController::poll cycle or by createIOcontroller
 * \param[in,out] asCmdReplyList  bIsRequest=true: append command to command list, bIsRequest=false: parse and remove 1st reply from replies
 * \param[in]     bIsRequest      type of call
 * \param[in]     pCtrl           pointer to controller
 * \param[in,out] iType           card type (change TYPE_GUESS_D or TYPE_GUESS_A to appropriate)
 * \param[in]     byCardNr        card number (starting with 1)
 * \param[in]     byChannel       channel number (starting with 1)
 * \param[out]    iIn             value of input(s)
 * \param[out]    iOut            value of output(s)
 */
bool phytronIO::pollIO(std::vector<std::string>& asCmdReplyList, bool bIsRequest, phytronController* pCtrl, IOTYPE &iType, epicsUInt8 byCardNr, epicsUInt8 byChannel, epicsInt32& iIn, epicsInt32& iOut)
{
  std::string sVal;
  bool bMCCbinary(false);
  if (pCtrl->iCtrlType_ == phytronController::TYPE_MCC)
  {
    if (byCardNr > 1)
      return false;
    switch (iType) // MCC-1/MCC-2 has fixed I/O
    {
      case TYPE_GUESS_D: iType = TYPE_DIO; bMCCbinary = true; break;
      case TYPE_GUESS_A: iType = TYPE_AI;                     break;
      case TYPE_DIO:                       bMCCbinary = true; break;
      default:                                                break;
    }
  }
  if (bIsRequest)
  {
    // generate requests
    switch (iType)
    {
      case TYPE_GUESS_D:
      case TYPE_DIO:
        asCmdReplyList.push_back(std::string("AG") + std::to_string(byCardNr) + "R");
        asCmdReplyList.push_back(std::string("EG") + std::to_string(byCardNr) + "R");
        break;
      case TYPE_GUESS_A:
      case TYPE_AIO:
        asCmdReplyList.push_back(std::string("DA") + std::to_string(byCardNr) + "." + std::to_string(byChannel));
        asCmdReplyList.push_back(std::string("AD") + std::to_string(byCardNr) + "." + std::to_string(byChannel));
        break;
      case TYPE_DI:
        asCmdReplyList.push_back(std::string("EG") + std::to_string(byCardNr) + "R");
        break;
      case TYPE_DO:
        asCmdReplyList.push_back(std::string("AG") + std::to_string(byCardNr) + "R");
        break;
      case TYPE_AI:
        if (pCtrl->iCtrlType_ == phytronController::TYPE_MCC)
          asCmdReplyList.push_back(std::string("AD") + std::to_string(byChannel) + "R");
        else
          asCmdReplyList.push_back(std::string("AD") + std::to_string(byCardNr) + "." + std::to_string(byChannel));
        break;
      case TYPE_AO:
        asCmdReplyList.push_back(std::string("DA") + std::to_string(byCardNr) + "." + std::to_string(byChannel));
        break;
      default:
        return false;
    }
    return true;
  }
  switch (iType) // check reply count
  {
    case TYPE_GUESS_D:
    case TYPE_GUESS_A:
    case TYPE_DIO:
    case TYPE_AIO:
      if (asCmdReplyList.size() < 2) // not enough values
        return false;
      break;
    case TYPE_DI:
    case TYPE_DO:
    case TYPE_AI:
    case TYPE_AO:
      if (asCmdReplyList.size() < 1) // not enough values
        return false;
      break;
    default:
      break;
  }
  switch (iType) // check for ACK/NAK and in case of TYPE_GUESS_? set correct type
  {
    case TYPE_GUESS_D:
      if (asCmdReplyList[0].substr(0, 1) == "\x06" && asCmdReplyList[1].substr(0, 1) == "\x06")
        iType = TYPE_DIO;
      else if (asCmdReplyList[0].substr(0, 1) != "\x06" && asCmdReplyList[1].substr(0, 1) != "\x06")
        return false;
      else
        iType = (asCmdReplyList[0].substr(0, 1) == "\x06") ? TYPE_DO : TYPE_DI;
      break;
    case TYPE_GUESS_A:
      if (asCmdReplyList[0].substr(0, 1) == "\x06" && asCmdReplyList[1].substr(0, 1) == "\x06")
        iType = TYPE_AIO;
      else if (asCmdReplyList[0].substr(0, 1) != "\x06" && asCmdReplyList[1].substr(0, 1) != "\x06")
        return false;
      else
        iType = (asCmdReplyList[0].substr(0, 1) == "\x06") ? TYPE_AO : TYPE_AI;
      break;

    case TYPE_DI:
    case TYPE_DO:
    case TYPE_AI:
    case TYPE_AO:
      if (asCmdReplyList[0].substr(0, 1) != "\x06") // no ACK
        return false;
      break;
    default:
      if (asCmdReplyList[0].substr(0, 1) != "\x06" || asCmdReplyList[1].substr(0, 1) != "\x06") // no ACK
        return false;
      break;
  }
  switch (iType) // handle output value
  {
    case TYPE_DIO:
    case TYPE_DO:
    case TYPE_AIO:
    case TYPE_AO:
      sVal = asCmdReplyList[0];
      sVal.erase(0, 1);
      if (bMCCbinary)
      {
        epicsInt32 iBit(1);
        iOut = 0;
        while (!sVal.empty())
        {
          switch (sVal.front())
          {
            case '0': break;
            case '1': iOut |= iBit; break;
            default: return false;
          }
          iBit <<= 1;
          sVal.erase(0, 1);
        }
      }
      else if (epicsParseInt32(sVal.c_str(), &iOut, 0, nullptr) != 0)
        return false;
      asCmdReplyList.erase(asCmdReplyList.begin(), asCmdReplyList.begin() + 1);
      break;
    default:
      break;
  }
  switch (iType) // handle input value
  {
    case TYPE_DIO:
    case TYPE_DI:
    case TYPE_AIO:
    case TYPE_AI:
      sVal = asCmdReplyList[0];
      sVal.erase(0, 1);
      if (bMCCbinary)
      {
        epicsInt32 iBit(1);
        iIn = 0;
        while (!sVal.empty())
        {
          switch (sVal.front())
          {
            case '0': break;
            case '1': iIn |= iBit; break;
            default: return false;
          }
          iBit <<= 1;
          sVal.erase(0, 1);
        }
      }
      else if (epicsParseInt32(sVal.c_str(), &iIn, 0, nullptr) != 0)
        return false;
      asCmdReplyList.erase(asCmdReplyList.begin(), asCmdReplyList.begin() + 1);
      break;
    default:
      break;
  }
  return true;
}
