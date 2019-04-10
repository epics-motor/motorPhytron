TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = phytron
# phytron.dbd will be created and installed
DBD += phytron.dbd

# phytron.dbd will be made up from these files:
phytron_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
phytron_DBD += asyn.dbd
phytron_DBD += drvAsynIPPort.dbd
#endif
phytron_DBD += motorSupport.dbd
phytron_DBD += phytronSupport.dbd

# Add all the support libraries needed by this IOC
phytron_LIBS += phytronAxisMotor
phytron_LIBS += motor
#ifdef ASYN
phytron_LIBS += asyn
#endif
#ifdef SNCSEQ
phytron_LIBS += seq pv
#endif

# phytron_registerRecordDeviceDriver.cpp derives from phytron.dbd
phytron_SRCS += phytron_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
phytron_SRCS_DEFAULT += phytronMain.cpp
phytron_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#phytron_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
phytron_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

