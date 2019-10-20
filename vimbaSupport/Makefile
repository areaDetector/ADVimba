TOP=../
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

INC += $(patsubst ../%, %, $(wildcard ../VimbaCPP/Include/*))
INC += $(patsubst ../%, %, $(wildcard ../VimbaC/Include/*))
INC += $(patsubst ../%, %, $(wildcard ../VimbaImageTransform/Include/*))
INC += $(patsubst ../%, %, $(wildcard ../Common/*))

# Note, the following files were manually copied from the Spinnaker distribution lib directories on Windows
# and from /usr/lib on Linux.
ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_INSTALLS    += ../os/windows-x64/VimbaC.lib
LIB_INSTALLS    += ../os/windows-x64/VimbaCPP.lib
LIB_INSTALLS    += ../os/windows-x64/VimbaImageTransform.lib

else ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
LIB_INSTALLS    += ../os/win32-x86/VimbaC.lib
LIB_INSTALLS    += ../os/win32-x86/VimbaCPP.lib
LIB_INSTALLS    += ../os/win32-x86/VimbaImageTransform.lib

else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_INSTALLS    += ../os/linux-x86_64/libVimbaC.so
LIB_INSTALLS    += ../os/linux-x86_64/libVimbaImageTransform.so

else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
LIB_INSTALLS    += ../os/linux-x86/libVimbaC.so
LIB_INSTALLS    += ../os/linux-x86/libVimbaImageTransform.so

endif

LIBRARY_IOC_WIN32 += VimbaCPP
LIBRARY_IOC_Linux += VimbaCPP

SRC_DIRS += $(TOP)/vimbaSupport/VimbaCPP/Source

#USR_CXXFLAGS_Linux += -fmax-errors=10

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

LIB_LIBS  += VimbaC

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

