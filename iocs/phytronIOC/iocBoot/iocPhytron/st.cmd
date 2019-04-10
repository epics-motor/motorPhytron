#!../../bin/linux-x86_64/phytron

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/phytron.dbd"
phytron_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=phytron:")

## 

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("phytron:")

# Boot complete
