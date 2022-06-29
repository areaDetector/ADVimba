TOP=../
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

INC += $(patsubst ../%, %, $(wildcard ../VimbaC/Include/*))
INC += $(patsubst ../%, %, $(wildcard ../VimbaImageTransform/Include/*))
INC += $(patsubst ../%, %, $(wildcard ../Common/*))

# Note, the following files were manually copied from the Spinnaker distribution lib directories on Windows
# and from /usr/lib on Linux.
ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_INSTALLS    += ../os/windows-x64/VimbaC.lib
LIB_INSTALLS    += ../os/windows-x64/VimbaImageTransform.lib
BIN_INSTALLS    += ../os/windows-x64/VimbaGigETL.cti
BIN_INSTALLS    += ../os/windows-x64/VimbaGigETL.xml
BIN_INSTALLS    += ../os/windows-x64/VimbaUSBTL.cti
BIN_INSTALLS    += ../os/windows-x64/VimbaUSBTL.xml

else ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
LIB_INSTALLS    += ../os/win32-x86/VimbaC.lib
LIB_INSTALLS    += ../os/win32-x86/VimbaImageTransform.lib
BIN_INSTALLS    += ../os/win32-x86/VimbaGigETL.cti
BIN_INSTALLS    += ../os/win32-x86/VimbaGigETL.xml
BIN_INSTALLS    += ../os/win32-x86/VimbaUSBTL.cti
BIN_INSTALLS    += ../os/win32-x86/VimbaUSBTL.xml

else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_INSTALLS    += ../os/linux-x86_64/libVimbaC.so
LIB_INSTALLS    += ../os/linux-x86_64/libVimbaImageTransform.so
BIN_INSTALLS    += ../os/linux-x86_64/VimbaGigETL.cti
BIN_INSTALLS    += ../os/linux-x86_64/VimbaGigETL.xml
BIN_INSTALLS    += ../os/linux-x86_64/VimbaUSBTL.cti
BIN_INSTALLS    += ../os/linux-x86_64/VimbaUSBTL.xml

else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
LIB_INSTALLS    += ../os/linux-x86/libVimbaC.so
LIB_INSTALLS    += ../os/linux-x86/libVimbaImageTransform.so
BIN_INSTALLS    += ../os/linux-x86/VimbaGigETL.cti
BIN_INSTALLS    += ../os/linux-x86/VimbaGigETL.xml
BIN_INSTALLS    += ../os/linux-x86/VimbaUSBTL.cti
BIN_INSTALLS    += ../os/linux-x86/VimbaUSBTL.xml

endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

