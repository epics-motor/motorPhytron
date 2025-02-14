# SPDX-License-Identifier: EPICS
TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
# Build the IOC application

LIBRARY_IOC += phytronAxisMotor

DBD += phytronSupport.dbd

# The following are compiled and added to the support library
phytronAxisMotor_SRCS += phytronAxisMotor.cpp
phytronAxisMotor_SRCS += phytronIOctrl.cpp
phytronAxisMotor_SRCS += phytronSupport.cpp

INC += phytronAxisMotor.h
INC += phytronIOctrl.h

phytronAxisMotor_LIBS += motor
phytronAxisMotor_LIBS += asyn
phytronAxisMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES

