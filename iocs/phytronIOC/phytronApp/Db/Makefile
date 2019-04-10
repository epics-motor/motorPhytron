TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

ifdef MOTOR_PHYTRON
DB_INSTALLS += $(MOTOR_PHYTRON)/db/Phytron_I1AM01.db
DB_INSTALLS += $(MOTOR_PHYTRON)/db/Phytron_MCM01.db
DB_INSTALLS += $(MOTOR_PHYTRON)/db/Phytron_motor.db
else
DB_INSTALLS += $(MOTOR)/db/Phytron_I1AM01.db
DB_INSTALLS += $(MOTOR)/db/Phytron_MCM01.db
DB_INSTALLS += $(MOTOR)/db/Phytron_motor.db
endif

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this
#DB += xxx.db

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

