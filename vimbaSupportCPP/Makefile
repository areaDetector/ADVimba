TOP=../
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

INC += $(patsubst ../%, %, $(wildcard ../VimbaCPP/Include/*))

LIBRARY_IOC_WIN32 += VimbaCPP
LIBRARY_IOC_Linux += VimbaCPP

SRC_DIRS += $(TOP)/vimbaSupportCPP/VimbaCPP/Source

USR_CXXFLAGS_WIN32 += -DWIN32
ifeq ($(STATIC_BUILD), YES)
  USR_CXXFLAGS_WIN32 += -DAVT_VMBAPI_CPP_LIB
else
  USR_CXXFLAGS_WIN32 += -DAVT_VMBAPI_CPP_EXPORTS
endif

LIB_SRCS += AncillaryData.cpp
LIB_SRCS += BaseFeature.cpp
LIB_SRCS += BasicLockable.cpp
LIB_SRCS += BoolFeature.cpp
LIB_SRCS += Camera.cpp
LIB_SRCS += Clock.cpp
LIB_SRCS += CommandFeature.cpp
LIB_SRCS += Condition.cpp
LIB_SRCS += ConditionHelper.cpp
LIB_SRCS += DefaultCameraFactory.cpp
LIB_SRCS += EnumEntry.cpp
LIB_SRCS += EnumFeature.cpp
LIB_SRCS += FeatureContainer.cpp
LIB_SRCS += Feature.cpp
LIB_SRCS += FileLogger.cpp
LIB_SRCS += FloatFeature.cpp
LIB_SRCS += Frame.cpp
LIB_SRCS += FrameHandler.cpp
LIB_SRCS += Interface.cpp
LIB_SRCS += IntFeature.cpp
LIB_SRCS += Mutex.cpp
LIB_SRCS += MutexGuard.cpp
LIB_SRCS += RawFeature.cpp
LIB_SRCS += Semaphore.cpp
LIB_SRCS += StringFeature.cpp
LIB_SRCS += VimbaSystem.cpp

LIB_LIBS += VimbaC

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

