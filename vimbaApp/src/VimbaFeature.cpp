// VimbaFeature.cpp
// Mark Rivers
// October 26, 2018

#include <VimbaFeature.h>

#include "VimbaCPP/Include/VimbaCPP.h"

static const char *driverName="VimbaFeature";

using namespace AVT;
using namespace AVT::VmbAPI;
using namespace std;

VimbaFeature::VimbaFeature(GenICamFeatureSet *set, 
                           std::string const & asynName, asynParamType asynType, int asynIndex,
                           std::string const & featureName, GCFeatureType_t featureType, CameraPtr pCamera)
                     
         : GenICamFeature(set, asynName, asynType, asynIndex, featureName, featureType),
         mCameraPtr(pCamera), mAsynUser(set->getUser())
{
    static const char *functionName = "VimbaFeature";
    
    if (VmbErrorSuccess == mCameraPtr->GetFeatureByName(featureName.c_str(), mFeaturePtr)) {
        mIsImplemented = true;
        VmbFeatureDataType dataType;
        checkError(mFeaturePtr->GetDataType(dataType), "VimbaFeature", "GetDataType");
        GCFeatureType_t GCFeatureType;
        switch (dataType) {
            case VmbFeatureDataInt: 
                GCFeatureType = GCFeatureTypeInteger;
                break;
            case VmbFeatureDataFloat: 
                GCFeatureType = GCFeatureTypeDouble;
                break;
            case VmbFeatureDataEnum: 
                GCFeatureType = GCFeatureTypeEnum;
                break;
            case VmbFeatureDataString: 
                GCFeatureType = GCFeatureTypeString;
                break;
            case VmbFeatureDataBool: 
                GCFeatureType = GCFeatureTypeBoolean;
                break;
            case VmbFeatureDataCommand: 
                GCFeatureType = GCFeatureTypeCmd;
                break;
            default:
                GCFeatureType = GCFeatureTypeUnknown;
                break;
        }
        if (mFeatureType == GCFeatureTypeUnknown) {
            mFeatureType = GCFeatureType;
        } else {
            if (featureType != GCFeatureType) {
                asynPrint(mAsynUser, ASYN_TRACE_ERROR,
                    "%s::%s error input feature type=%d != Vimba feature type=%d for featurename=%s\n",
                    driverName, functionName, featureType, GCFeatureType, featureName.c_str());
            }
        }
        if (mFeatureType == GCFeatureTypeUnknown) {
            asynPrint(mAsynUser, ASYN_TRACE_ERROR,
                "%s::%s error unknown feature type for featureName=%s\n",
                driverName, functionName, featureName.c_str());
        }
    } else {
        mIsImplemented = false;
    }
}

inline asynStatus VimbaFeature::checkError(VmbErrorType error, const char *functionName, const char *VMBFunction)
{
    if (VmbErrorSuccess != error) {
        asynPrint(mAsynUser, ASYN_TRACE_ERROR,
            "%s:%s: ERROR calling %s error=%d\n",
            driverName, functionName, VMBFunction, error);
        return asynError;
    }
    return asynSuccess;
}

bool VimbaFeature::isImplemented() { 
    return mIsImplemented; 
}

bool VimbaFeature::isAvailable() {
    // Vimba does not support isAvailable.  We simulate it by checking if it is readable or writable.
    bool readable;
    bool writable;
    if (!mIsImplemented) return false;
    checkError(mFeaturePtr->IsReadable(readable), "isAvailable", "IsReadable");
    checkError(mFeaturePtr->IsWritable(writable), "isAvailable", "IsWritable");
    return (readable || writable);
}

bool VimbaFeature::isReadable() { 
    bool value; 
    if (!mIsImplemented) return false;
    checkError(mFeaturePtr->IsReadable(value), "isReadable", "IsReadable");
    return value;
}

bool VimbaFeature::isWritable() { 
    bool value; 
    if (!mIsImplemented) return false;
    checkError(mFeaturePtr->IsWritable(value), "isWritable", "IsWritable");
    return value;
}

int VimbaFeature::readInteger() {
    VmbInt64_t value; 
    if (!mIsImplemented) return 0;
    checkError(mFeaturePtr->GetValue(value), "readInteger", "GetValue");
    return value;
}

int VimbaFeature::readIntegerMin() {
    VmbInt64_t min, max; 
    if (!mIsImplemented) return 0;
    checkError(mFeaturePtr->GetRange(min, max), "readIntegerMin", "GetRange");
    return min;
}

int VimbaFeature::readIntegerMax() {
    VmbInt64_t min, max; 
    if (!mIsImplemented) return 0;
    checkError(mFeaturePtr->GetRange(min, max), "readIntegerMax", "GetRange");
    return max;
}

int VimbaFeature::readIncrement() { 
    VmbInt64_t inc; 
    if (!mIsImplemented) return 0;
    checkError(mFeaturePtr->GetIncrement(inc), "readIncrement", "GetIncrement");
    return inc;
}

void VimbaFeature::writeInteger(int value) { 
    if (!mIsImplemented) return;
    checkError(mFeaturePtr->SetValue(value), "writeInteger", "SetValue");
}

bool VimbaFeature::readBoolean() {
    bool value;
    if (!mIsImplemented) return false;
    checkError(mFeaturePtr->GetValue(value), "readBoolean", "GetValue");
    return value;
}

void VimbaFeature::writeBoolean(bool value) { 
    if (!mIsImplemented) return;
    checkError(mFeaturePtr->SetValue(value), "writeBoolean", "SetValue");
}

double VimbaFeature::readDouble() { 
    double value;
    if (!mIsImplemented) return 0.0;
    checkError(mFeaturePtr->GetValue(value), "readDouble", "GetValue");
    return value;
}

void VimbaFeature::writeDouble(double value) { 
    if (!mIsImplemented) return;
    checkError(mFeaturePtr->SetValue(value), "writeDouble", "SetValue");
}

double VimbaFeature::readDoubleMin() {
    double min, max; 
    if (!mIsImplemented) return 0.0;
    checkError(mFeaturePtr->GetRange(min, max), "readDoubleMin", "GetRange");
    return min;
}

double VimbaFeature::readDoubleMax() {
    double min, max; 
    if (!mIsImplemented) return 0.0;
    checkError(mFeaturePtr->GetRange(min, max), "readDoubleMax", "GetRange");
    return max;
}

int VimbaFeature::readEnumIndex() {
    VmbInt64_t value;
    if (!mIsImplemented) return 0;
    checkError(mFeaturePtr->GetValue(value), "readEnumIndex", "GetValue"); 
    return value;
}

void VimbaFeature::writeEnumIndex(int value) {
    if (!mIsImplemented) return;
    checkError(mFeaturePtr->SetValue(value), "writeEnumIndex", "SetValue"); 
}

std::string VimbaFeature::readEnumString() {
    return ""; 
}

void VimbaFeature::writeEnumString(std::string const &value) { 
}

std::string VimbaFeature::readString() {
    std::string value; 
    if (!mIsImplemented) return "";
    checkError(mFeaturePtr->GetValue(value), "readString", "GetValue");
    return value; 
}

void VimbaFeature::writeString(std::string const & value) { 
    if (!mIsImplemented) return;
    checkError(mFeaturePtr->SetValue(value.c_str()), "writeString", "SetValue"); 
}

void VimbaFeature::writeCommand() { 
    if (!mIsImplemented) return;
    if (checkError(mFeaturePtr->RunCommand(), "writeCommand", "RunCommand")) {
        bool bIsCommandDone = false;
        do {
            if (checkError(mFeaturePtr->IsCommandDone(bIsCommandDone), "writeCommand", "IsCommandDone")) {
                break;
            }
        } while ( false == bIsCommandDone );
    }
}

void VimbaFeature::readEnumChoices(std::vector<std::string>& enumStrings, std::vector<int>& enumValues) {
    EnumEntryVector entries;
    if (!mIsImplemented) return;
    checkError(mFeaturePtr->GetEntries(entries), "readEnumChoices", "GetEntries");
    int numEnums = (int)entries.size();
    bool available;
    VmbInt64_t value;
    std::string str;
    for (int i=0; i<numEnums; i++) {
        checkError(entries[i].GetValue(value), "readEnumChoices", "GetValue");
        checkError(mFeaturePtr->IsValueAvailable(value, available), "readEnumChoices", "IsValueAvailable");
        if (available) {
            checkError(entries[i].GetDisplayName(str), "readEnumChoices", "GetDisplayName");
            enumStrings.push_back(str);
            enumValues.push_back(value);
        }
    }
}

