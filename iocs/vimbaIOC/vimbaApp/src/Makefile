TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

PROD_NAME = vimbaApp
PROD_IOC_WIN32 += $(PROD_NAME)
PROD_IOC_Linux += $(PROD_NAME)

ifeq ($(STATIC_BUILD), YES)
  USR_CXXFLAGS_WIN32 += -DAVT_VMBAPI_CPP_LIB
endif

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += ADVimbaSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
$(PROD_NAME)_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

# Add locally compiled object code
PROD_LIBS += ADVimba
PROD_LIBS += ADGenICam 
PROD_LIBS_WIN32 += VimbaCPP VimbaC VimbaImageTransform
PROD_LIBS_Linux += VimbaCPP 
PROD_SYS_LIBS_Linux += VimbaC VimbaImageTransform

include $(ADCORE)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

