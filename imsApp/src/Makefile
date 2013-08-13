# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
# ADD MACRO DEFINITIONS AFTER THIS LINE

# The following are used for debugging messages.
USR_CXXFLAGS += -DDEBUG

DBDINC       += imsRecord.h

DBD          += imsSupport.dbd

LIBRARY_IOC  += ims

# The following are required for all ims record configurations.
ims_SRCS     += imsRecord.cc

ims_LIBS     += asyn
ims_LIBS     += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
# ADD RULES AFTER THIS LINE
