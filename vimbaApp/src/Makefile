TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#LIBRARY_IOC_WIN32 += ADVimba
#LIBRARY_IOC_Linux += ADVimba

#LIB_SRCS += ListCameras.cpp

#LIB_LIBS_Linux  += Spinnaker

DBD += ADVimbaSupport.dbd

PROD_IOC_Linux += ListCameras
PROD_IOC_WIN32 += ListCameras
ListCameras_SRCS += ListCamerasMain.cpp ListCameras.cpp

PROD_LIBS_WIN32 += VimbaCPP VimbaC

# We need to list this as a SYS_LIBS when building statically because it is only available as a .so, not .a
PROD_SYS_LIBS_Linux += VimbaCPP VimbaC

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

