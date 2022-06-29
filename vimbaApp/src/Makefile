TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC_WIN32 += ADVimba
LIBRARY_IOC_Linux += ADVimba

LIB_SRCS += VimbaFeature.cpp
LIB_SRCS += ADVimba.cpp

LIB_LIBS += ADGenICam

ifeq ($(STATIC_BUILD), YES)
  USR_CXXFLAGS_WIN32  += -DAVT_VMBAPI_CPP_LIB
  # We need to list this as a SYS_LIBS when building statically because it is only available as a .so, not .a
  LIB_SYS_LIBS_Linux  += VimbaCPP VimbaC VimbaImageTransform
  PROD_SYS_LIBS_Linux += VimbaCPP VimbaC VimbaImageTransform
else
  LIB_LIBS_Linux  += VimbaCPP VimbaC VimbaImageTransform
  PROD_LIBS_Linux += VimbaCPP VimbaC VimbaImageTransform
endif
LIB_LIBS_WIN32  += VimbaCPP VimbaC VimbaImageTransform
PROD_LIBS_WIN32 += VimbaCPP VimbaC VimbaImageTransform

DBD += ADVimbaSupport.dbd

PROD_IOC_Linux += ListCameras
PROD_IOC_WIN32 += ListCameras
ListCameras_SRCS += ListCamerasMain.cpp ListCameras.cpp

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

