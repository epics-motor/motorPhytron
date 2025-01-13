********************************************************************************
Phytron phyMOTION I1AM01, I1AM02, I1EM01, I1EM02 Stepper Motor Controller
Phytron MCC-1, MCC-2 Stepper Motor Controller with I/O
Phytron DIOM01.1, AIOM01.1 Digital and Analog I/O Controller
Asyn Driver Documentation

SPDX-License-Identifier: EPICS
Authors: Tom Slejko, Bor Marolt, Cosylab d.d.
		tom.slejko@cosylab.com
		bor.marolt@cosylab.com
	 Lutz Rossa, Helmholtz-Zentrum Berlin fuer Materialien und Energy GmbH
		<rossa@helmholtz-berlin.de>
	 Will Smith, Helmholtz-Zentrum Berlin fuer Materialien und Energy GmbH
		<william.smith@helmholtz-berlin.de>
	 Bernhard Kuner, Helmholtz-Zentrum Berlin fuer Materialien und Energy GmbH
		<bernhard.kuner@helmholtz-berlin.de>

********************************************************************************
Table of contents:
- Controller and axis configuration
    - Example Application
    - New Applicaion
- Database
    - Supported I1AM01, I1AM02, I1EM01, I1EM02 Features
         - Initialization records
         - Status
         - Homing
         - Reset
         - I1AM01, I1AM02, I1EM01, I1EM02 parameters
    - Supported MCM01 Features
    - Supported MCC-1, MCC-2 Features
    - Motor record
    - List of remaining I1AM01, I1AM02, I1EM01, I1EM02, MCC-1, MCC-2 parameters
      handled by the motor record, internally by controller, or not applicable
- GUI
- Asyn option interface


Controller and axis configuration
=================================
Example application:
--------------------
Motor record includes an example Phytron application. Before using it do:

cd $(MOTOR_ROOT)/iocBoot/iocWithAsyn/

where $(MOTOR_ROOT) is the location of the motor record. Open st.cmd.phytron and
modify controller's IP address. If using serial port instead of ethernet, read 
"New application" section, on how to configure the serial port. Remove or add 
calls to phytronCreateAxis according to the number of axes that are to be used 
(read "New application" section on how to use phytronCreateAxis). 
Once st.cmd is configured, run:

../../bin/linux-x86_64/WithAsyn st.cmd.phytron

New application:
----------------
This section describes how to properly configure epics application in order to
use phytronAxisMotor driver.

The following dbd files and libraries must be added to the $(APP)/src/Makefile:
test_DBD += asyn.dbd
test_DBD += motorSupport.dbd
test_DBD += drvAsynIPPort.dbd
test_DBD += drvAsynSerialPort.dbd  # for use of USB communication
test_DBD += phytron.dbd

test_LIBS += asyn
test_LIBS += motor
test_LIBS += phytronAxisMotor

An example motor.substitutions.phytron and st.cmd.phytron files are located in 
the $(MOTOR_RECORD)/iocBoot/iocWithAsyn/ directory.

********************************************************************************
WARNING: For the controller to work properly all three database files (
Phytron_I1AM01.db, Phytron_MCM01.db and Phytron_motor.db) must be used.
********************************************************************************

Start up script must perform the following:
Before configuring the controller the user must create an asyn port by running 
drvAsynIPPortConfigure or drvAsynSerialPortConfigure in order to create a 
communication interface to Phytron's MCM Unit which controlls the I1AM01,
I1AM02, I1EM01 and I1EM02 modules or Phytron MCC Unit which controlls the MCC-1
or MCC-2 axes. Below any type of I1AM01 means also the other types I1AM02,
I1EM01, I1EM02, MCC-1 and MCC-2.

If serial port is used for communication with the controller, baud rate, number
of data bits, parity, number of stop bits and End Of Line character must be set,
e.g.:

drvAsynSerialPortConfigure ("testRemote","/dev/ttyUSB0")
asynOctetSetInputEos("testRemote",0,"\3")
asynSetOption ("testRemote", -1, "baud", "115200")
asynSetOption ("testRemote", -1, "bits", "8")
asynSetOption ("testRemote", -1, "parity", "none")
asynSetOption ("testRemote", -1, "stop", "1")

If ethernet is used, the IP and port must be set, e.g.:
drvAsynIPPortConfigure("testRemote","10.5.1.181:22222",0,0,1)

Phytron (phyMOTION MCM) controller port is configured (and connected to
previously created asyn port) by running the following iocsh function:

phytronCreateController(const char *phytronPortName, const char *asynPortName,
                        int movingPollPeriod, int idlePollPeriod,
                        double timeout, int noResetAtBoot)

or Phytron (MCC) controller port is configured (and connected to previously
created asyn port) by running the following iocsh function:

phytronCreateMCC(const char *phytronPortName, const char *asynPortName,
                 int address, int movingPollPeriod, int idlePollPeriod,
                 double timeout, int noResetAtBoot)

- phytronPortName: Name of the particular MCM or MCC unit.
- asynPortName: Name of the previously configured asyn port (MCM/MCC interface)
- address (MCC only): hardware address switch value 0..15
- movingPollPeriod: The time between polls when any axis is moving in ms
- idlePolPeriod: The time between polls when no axis is moving in ms
- Timeout: Milliseconds before timeout for I/O requests
- noResetAtBoot: if 1 then the controller is not reset at boot. If unset or 0 it is

where poll reads the basic axis status, e.g. position of the motor and of the
encoder, checks if axis is in movement, checks if motor is at the limit
switch, ...

Once the phytron controller is configured, user can initialize axes by running

phytronCreateAxis(const char* phytronPortName, int module, int axis)
- phytronPortName: Previously defined name of the MCM/MCC unit
- module: index of the motor module connected to the MCM, MCC: use 0
- axis: index of the axis on the motor module (starting with 1)

Module index and axis index compose the axis asyn ADDR (ADDR macro) used in the
motor.substitutions file. 

PhytronCreateAxis must be called for each axis intended to be used.

If the axis has a brake and it should be triggered by a digital output, the
command phytronBrakeOutput could be used to configure this:

phytronBrakeOutput(const char* phytronPortName, float fAxis, float fOutput,
                   int bDisableMotor, double dEngageTime, double dReleaseTime)
- phytronPortName: Previously defined name of the MCM/MCC unit
- fAxis:   <module>.<axis> as float number for axis selection
- fOutput: <module>.<output> as float number for digital output selection
                   or 0.0 to disable, negative value inverts output
- bDisableMotor:   0=keep motor enabled, 1=disable motor output, when idle
- dEngageTime:     time is milliseconds to engage brake (max. 10 sec)
                   and the motor is disabled after this time
- dReleaseTime:    time is milliseconds to release brake (max. 10 sec)
                   and motor is started after this time

Note: The motor thread is blocked for the wait time and no other axis nor the
      controller is updated while waiting. Keep this time as short as possible!
Note: The brake support inside EPICS records may overwrite this.

********************************************************************************
WARNING: For every axis, the user must specify it's address (ADDR macro) in the 
motor.substitutions file for Phytron_motor.db and PhytronI1AM01.db files.
The address is composed of the I1AM01 module index and the axis index. If, for 
example, the startup script configures the following axis:

drvAsynIPPortConfigure("testRemote","10.5.1.181:22222",0,0,1)
phytronCreateController ("phyMotionPort", "testRemote", 100, 100, 1000)
phytronCreateAxis("phyMotionPort", 2, 1)

It's asyn address that must be specified in the substitutions file is 21,

PORT macro in the substitution file must match the phytronPortName, which is in
the example above "phyMotionPort"
********************************************************************************

Example st.cmd:
---------------
#!../../bin/linux-x86_64/test

## You may have to change test to something else
## everywhere it appears in this file

< envPaths

## Register all support components
dbLoadDatabase("../../dbd/test.dbd",0,0)
test_registerRecordDeviceDriver(pdbbase) 

drvAsynIPPortConfigure("testRemote","10.5.1.181:22222",0,0,1)

phytronCreateController ("phyMotionPort", "testRemote", 100, 100, 1000)

#phytronCreateAxis(phytronPort, module, axis)
phytronCreateAxis("phyMotionPort", 1, 1)
phytronCreateAxis("phyMotionPort", 2, 1)

cd $(MOTOR)/iocBoot/iocWithAsyn/
dbLoadTemplate "motor.substitutions.phytron"

iocInit()

Database:
=========
All three database files (Phytron_I1AM01.db, Phytron_MCM01.db, Phytron_motor.db)
in $(MOTOR_ROOT)/motorApp/Db are expanded by motor.substitutions.phytron file in
$(MOTOR_ROOT)/iocBoot/iocWithAsyn

********************************************************************************
WARNING: For the controller to work properly all three database files (
Phytron_I1AM01.db, Phytron_MCM01.db and Phytron_motor.db) must be used.
********************************************************************************

===============================================
I1AM01 Controller Features - Phytron_I1AM01.db:
===============================================
Initialization:
---------------
Database contains 2 initialization records $(P)$(M)-INIT-AXIS-1/2 which are used
to initalize output (_SET) records. Initialization values are defined in the 
motor.substitutions file.

Status:
-------
Database contains several status records. Ai record $(P)$(M)-STATUS_ reads the 
status value from the I1AM01 module, 2 mbbiDirect records ($(P)$(M)-STATUS-1/2) 
are provided to set the status bits - 23 bi records are provided to parse these
status bits. An additional calc record is provided ($(P)$(M)-AXIS-ERR) which
sets it's value to 1 if any of the error bits are set (if only notification 
bits are set, e.g. axis-initialized, the value of $(P)$(M)-AXIS-ERR is 0).

Homing:
-------
RECORDS mbbo/mbbi: $(P)$(M)-HOMING_SET/_GET are used to set and readback the 
type of homing to be used. Homing is executed with the use of motor record's 
HOMF, HOMR fields. The following options are available for $(P)$(M)-HOMING_SET, 
please keep in mind that offsets defined by I1AM01 parameters P11 and P12 (see 
phylogic-en.pdf and see below for records corresponding to P11 and P12) affect
the final position after the homing procedure:
<- recordValue (Homing description); Phytron commands for HOMF and HOMR>

- Limit (Home on limit switches); HOMF - m.aR+, HOMR - m.aR-

- Center (Home on center switch); HOMF - m.aR+C, HOMR - m.aR-C
COMMENT: If limit switch is reached, the motor will stop and start moving in the
opposite direction until center switch is reached

- Encoder (Home on encoder zero pulse); HOMF - m.aR+I, HOMR - m.aR-I

- Limit-Encoder (Got to limit switch, than encoder zero pulse); 
        HOMF - m.aR+^I, HOMR - m.aR-^I

- Center-Encoder (Go to the center switch than to encoder zero pulse)
        HOMF - m.a.R+C^I, HOMR - m.aR-C^I
COMMENT: If limit switch is reached before center switch, the motor will stop 
and start moving in the opposite direction until center switch is reached

- Reference-Center (Driving on a reference signal to center)
        HOMF - m.aRC+, HOMR - m.aRC-
COMMEN: If limit switch is reached, controller goes to axis error state

- Ref-Center-Encoder (Driving on a reference signal to center and then to 
encoder zero pulse)
        HOMF - m.aRC+^I, HOMR - m.aRC-^I

Brake support:
--------------
RECORDS
  $(P)$(M)-BRAKE-OUTPUT
    Digital output used for a brake as floating point "<module>.<output>".
    If the value is positive, the output is set while move and reset on idle
    state. If the value is negative, the output is inverted. To disable,
    set it to 0.0, which is the default.
  $(P)$(M)-DISABLE_MOTOR
    Set to non-zero to disable motor output on idle state.
  $(P)$(M)-BRAKE-ENGAGE-TIME
    Time in seconds after engaging the brake before disabling the motor.
  $(P)$(M)-BRAKE-RELEASE-TIME
    Time in seconds after enabling the motor before releasing the brake.

Please do not spend too much time for switching the brake, because the thread
is blocked for the wait time and no other axis nor the controller is updated
while waiting. Keep this time as short as possible!

Reset:
------
Two records are provided. Record $(P)$(M)-RESET executs Phytron command: m.aC,
where m is controller index and a is the axis index.

$(P)$(M)-RESET-STATUS resets the axis status by executing the Phytron command: 
SECm.a, where m is controller index and a is the axis index.

If user resets the MCM unit, $(P)$(M)-REINIT_ record is triggered. This record
process the initialization records $(P)$(M)-INIT-AXIS-1/2 after a DELAY time
defined by the macro DLY in the substitutions file.


The following I1AM01 parameters are exposed as additional EPICS records:
------------------------------------------------------------------------
Param. index: feature;
Record name(s):
Comment:
-------------------------------------
P01: Type of movement;
RECORDS mbbo/mbbi: $(P)$(M)-MOVE-TYP_SET/_GET; 
COMMENT: Software limits are always monitored  by the motor record, so only 2 
types of movements are possible:
    - Rotational: HW limit/center switches are ignored
    - HW switches are monitored
-------------------------------------
P11: M0P offset for limit switch in direction towards LIMIT-;
RECORDS ao/ai: $(P)$(M)-POS-OFFSET_SET/_GET
-------------------------------------
P12: M0P offset for limit switch in direction towards LIMIT+;
RECORDS ao/ai: $(P)$(M)-NEG-OFFSET_SET/_GET
-------------------------------------
P13: Time lapse during initialization
RECORDS ao/ai: $(P)$(M)-INIT-TIMEOUT_SET/_GET
-------------------------------------
P16: Time lapse after positioning
RECORDS ao/ai: $(P)$(M)-POS-TIMEOUT_SET/_GET
-------------------------------------
P17: Defines when to use boost current 
RECORDS mbbo/mbbi: $(P)$(M)-BOOST_SET/_GET
-------------------------------------
P26: Encoder data transfer rate (ONLY FOR SSI)
RECORDS mbbo/mbbi: $(P)$(M)-ENC-RATE_SET/_GET
-------------------------------------
P27: Limit switch type
RECORDS mbbo/mbbi: $(P)$(M)-SWITCH-TYP_SET/_GET
-------------------------------------
P28: Power stage off/on
RECORDS bo/bi: $(P)$(M)-PWR-STAGE-MODE_SET/_GET
-------------------------------------
P34: Encoder type
RECORDS mbbo/mbbi: $(P)$(M)-ENC-TYP_SET/_GET
-------------------------------------
P35: Encoder resolution - for SSI and EnDat encoders
RECORDS ao/ai: $(P)$(M)-ENC-RES_SET/_GET
-------------------------------------
P36: Encoder function: If 1, Encoder position is continuously compared to the 
motor position
RECORDS: bo/bi: $(P)$(M)-ENC-FUNC_SET/_GET
COMMENT: _SET is processed when $(P)$(M)-ENC-SFI_SET corresponding to parameter 
P37 is processed. If P37 > 0, then P36 is set to 1, else it is 0.
-------------------------------------
P37: If P36 is set to 1, controller will stop the motion when the difference
between motor and encoder position is > P37.
RECORDS: bo/bi: $(P)$(M)-ENC-SFI_SET/_GET
COMMENT: If value is set to > 0, then P36 is set to 1. If P37=0, then P36=0
-------------------------------------
P38: Encoder direction of rotation: Positive/Negative
RECORDS: bo/bi: $(P)$(M)-ENC-DIR_SET/_GET
-------------------------------------
P39: Encoder direction of rotation
RECORDS: bo/bi: $(P)$(M)-ENC-DIR_SET/_GET
-------------------------------------
P40: Stop current in mA. Can be set in 10 mA increments
RECORDS: ao/ai: $(P)$(M)-STOP-CURRENT_SET/_GET
-------------------------------------
P41: Run current in mA. Can be set in 10 mA increments
RECORDS: ao/ai: $(P)$(M)-RUN-CURRENT_SET/_GET
-------------------------------------
P42: Boost current in mA. Can be set in 10 mA increments
RECORDS: ao/ai: $(P)$(M)-BOOST-CURRENT_SET/_GET
-------------------------------------
P43: Current hold time in ms.
RECORDS: ao/ai: $(P)$(M)-CURRENT-DELAY_SET/_GET
-------------------------------------
P49: Power stage temperature
RECORDS: ai: $(P)$(M)-PS-TEMPERATURE
-------------------------------------
P53: Power stage monitoring
RECORDS: bo/bi: $(P)$(M)-PS-MONITOR/_GET
-------------------------------------
P54: Motor temperature
RECORDS: ai: $(P)$(M)-MOTOR-TEMP



============================================
MCM01 Controller Features - Phytron_MCM01.db:
============================================
This database file contains records for reading MCM01 status and to reset the 
MCM01 module.

Ai record $(P)-STATUS_ reads the status value from the MCM01 module, a 
mbbiDirect record is provided to set the status bits - 16 bi records are 
provided to parse these bits. An additional calc record is provided 
($(P)-CON-ERR) which sets it's value to 1 if any of the error bits are set (if 
only noticiation bits are set, e.g. terminal-activated, the value of 
$(P)-CON-ERR is 0)

Bo record $(P)-RESET resets the MCM01 controller, every time it is processed. 
$(P)-RESET_ alternates between 0 and 1, so the monitor is posted on every reset
and the axis REINIT_ record can trigger the initialization procedures.

================================
Motor Record - Phytron_motor.db:
================================
This database file is similar to basic_asyn_motor.db, the only difference is, 
that 2 additional fields are defined:
  - field(ERES,"$(ERES)") - encoder resolution
  - field(VMAX,"$(VMAX)") - maximum velocity


===============================================================================
List of remaining I1AM01 parameters handled by the motor record,  internally by 
controller, or not applicable
===============================================================================
P02 (Units of movemnt) - Always set to step - unit conversion is done within the
motor record.

P03 (Conversion factor for the thread) - Always set to 1 - unit conversion is 
done within the motor record.

P04 (Start/stop frequency) - Set by phytronAxis::move, before move is executed.

P07 (Emergency stop ramp) - Set by phytronAxis::stop, before stop is executed.

P08 (Initialization run frequency) - Set by phytronAxis::home, before homing is 
executed.

P09 (Ramp M0P) - Set by phytronAxis::home, before homing is executed.

P10 (Run frequency for leaving the limit switch) - Set by phytronAxis::home, 
before homing is executed

P14 (Run frequency during program operation) - Set by phytronAxis::move, before
move is executed

P15 (Ramp for run frequency) - Set by phytronAxis::move, before move is executed

P18 (Used internally by controller)

P19 (Encoder deviation M0P counter)

P20 (Mechanical 0 counter) - read by phytronAxis::poll to determine motor 
position

P21 (Absolute counter)

P22 (Encoder counter) - read by phytronAxis::poll to determine encoder position

P23 and P24 - Software limit switches - these parameters are ignored, because 
motor record handles software limits

P25 (Compensation for play) - Ignored, because motor records handles backlash 
corrections

P29 (Not used (by controller))

P30 and P31 (For I4XM01 only)

P32 and P33 (Not used (by controller))

P44 (For I4XM01 only)

P46, P47, P48 (Not used (by controller))

P50 and P51 (For I4XM01 only)

P52 (Internally used for trigger position)


GUI:
====
The follwoing CSS BOY GUI screens are located in motorApp/op/opi:

PhytronMain.opi
---------------
Motor record name is usually defined with 2 macros:
record(motor, "$(P)$(M)")

The value of the macro $(IOC) of the PhytronMain.opi screen must match the value
of the macro $(P) which is defined in motor.substitutions.

PhytronMain.opi has 2 action buttons to open screen PhytronI1AM01.opi for motor 
$(IOC):$(M), where the default values for $(M) are "m1" for the left button  
and "m2" for the right button. User must modify the values of macros $(IOC) and
$(M) according to the macro values in motor.subsititutions.

PhytronMain.opi also contains a CONTROLLER STATUS line. The LED displays the 
value of record $(P)-CON-ERR (see section MC01 Controller Features). Action 
button "MORE" opens the screen PhytronMCM01Status.opi, action button "RESET"
resets the MCM01 controller and action button "RESET CONTROLLER" resets 
controller status.

PhytronI1AM01.opi
-----------------
Status line contains a LED which displays the value of the record 
$(P)$(M)-AXIS-ERR, and an action button which displays the PhytronI1AM01.opi
screen. Action buttons "RESET AXIS" and "RESET STATUS" are provided.

The remaining widgets are used to set and read back the values of supported 
I1AM01 parameters (see section I1AM01 Controller Features).

PhytronI1AM01Status.opi
-----------------------
Contains an LED for each of the 23 bi records parsing the $(P)$(M)-STATUS-1/2
mbbiDirect records.

PhytronMCM01Status.opi
----------------------
Contains an LED for each of the 16 bi records parsing the $(P)-STATUS-BITS_ 
mbbiDirect record


Asyn option interface:
======================
This support wants to be backward compatible to older versions as much as
possible. It supports setting additional options via the asyn option interface
with iocsh commands "asynSetOption" or "asynShowOption". ADDR=0 means the
controller and ADDR=1... means a specific axis, which was created using
"phytronCreateAxis" above:

- asynSetOption(PORT, ADDR, "pollMethod", ...)
  This option allows to speed up communication with MCM controllers. Is is
  possible to set up different methods for different axes and the controller
  (ADDR=0). These methods are available:
  * "serial" or "no-parallel" or "not-parallel" or "old": this is the default
    method. It is compatible but slow and does a handshake (request + reply)
    for every command,
  * "axis" or "axis-parallel": this does some parallel command execution (one
    handshake for every axis) and is supported on many controllers,
  * "parallel" or "controller-parallel": this tries to do one handshake for all
    axes at a time. This is the fastest communication and supported on many
    controllers,
  * "default" or "standard": this is for a single axis only and uses the
    setting for the controller.
  Every handshake to a TCP-connected controller takes around 10ms, so the
  serial method takes 40ms per axis, the axis-parallel method takes 10ms per
  axis and the fastest controller-parallel method takes 10ms for _all_ axes.
  Note: This option is not supported by the MCC controllers.
- asynShowOption(PORT, ADDR, "pollMethod") shows the actual value as numeric
  value with text

- asynSetOption(PORT, ADDR, "fakeHomedEnable", ...)
  This option enables support for faked "homed" status of all axes. It uses
  the Phytron controller registers 1001 to 1020, which are normally zeroed
  after reset. The "homed" status is reset with any home command and will be
  set, if the motor record sets a new position (e.g. AUTOSAVE). This is or-ed
  together with the hardware "homed" status. The register 1001 will also be
  used to detect restarted motor controllers.
  Allowed values are "true" or "false" (default).
- asynShowOption(PORT, ADDR, "fakeHomedEnable") shows the actual value.

- asynShowOption(PORT, ADDR, "fakeHomedCache") shows the actual value of the
  software faked HOMED bits for all axes as list of decimal values. This is
  a debugging helper. The value index is (ADDR/10-1) and every bit corresponds
  to ADDR modulo 10 of the axes. Bit0 of 1st value is always set for reset
  detection.

- asynSetOption(PORT, ADDR, "allowExitOnError", ...)
  This option allows to exit the IOC in case of serious errors, e.g. a
  restarted motor controller. Normally an IOC should be started via procServ,
  which will restart it.
  Allowed values are "true" or "false" (default).
- asynShowOption(PORT, ADDR, "allowExitOnError") shows the actual value.
