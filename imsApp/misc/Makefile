# Makefile for copying streamDevice protocol files during build.
# Author: Jason Tiller

T_A=$(EPICS_HOST_ARCH)
TOP=../..
include $(TOP)/configure/CONFIG

######################################################################
# Specify what files should be copied and where.

ifeq ($(BASE_MODULE_VERSION),R3.14.12-0.4.0)
# What protocol files should be copied as part of the build.
COPY_FILES += ims_bootup.mcode

# What subdirectory should hold the protocol files.
COPY_SUBDIR = misc

else
# What protocol files should be copied as part of the build.
PROTO += ims_bootup.mcode

# What subdirectory should hold the protcol files.
INSTALL_PROTO = $(INSTALL_LOCATION)/misc
endif

# Uncomment this if you only want to copy when *not* installing to
# your development area.  Useful for iocBoot, where the source and
# target directories are actually the same.
# COPY_ONLY_IF_NON_LOCAL=1

######################################################################
# Don't change these.
include $(TOP)/configure/RULES
ifeq ($(BASE_MODULE_VERSION),R3.14.12-0.4.0)
include $(TOP)/configure/RULES.copy
endif

######################################################################
# Add additional RULES files, if necessary.
