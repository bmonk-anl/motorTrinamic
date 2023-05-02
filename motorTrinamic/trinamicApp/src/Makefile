TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC = Trinamic 

# motorRecord.h will be created from motorRecord.dbd
DBD += TrinamicSupport.dbd

# The following are compiled and added to the Support library
SRCS += TrinamicDriver.cpp 

Trinamic_LIBS += motor asyn
Trinamic_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

