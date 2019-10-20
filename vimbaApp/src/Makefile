TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC_WIN32 += ADVimba
LIBRARY_IOC_Linux += ADVimba

ifeq ($(STATIC_BUILD), YES)
  USR_CXXFLAGS_WIN32 += -DAVT_VMBAPI_CPP_LIB
endif

LIB_SRCS += VimbaFeature.cpp
LIB_SRCS += ADVimba.cpp

LIB_LIBS  += VimbaCPP VimbaC

DBD += ADVimbaSupport.dbd

PROD_LIBS_WIN32 += VimbaCPP VimbaC

# We need to list this as a SYS_LIBS when building statically because it is only available as a .so, not .a
PROD_SYS_LIBS_Linux += VimbaCPP VimbaC

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

