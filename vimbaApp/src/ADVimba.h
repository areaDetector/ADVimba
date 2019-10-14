#ifndef ADVMBINNAKER_H
#define ADVMBINNAKER_H

#include <epicsEvent.h>
#include <epicsMessageQueue.h>

#include <ADGenICam.h>
#include <VimbaFeature.h>

#include "VimbaCPP/Include/VimbaCPP.h"

using namespace AVT;
using namespace AVT::VmbAPI;
using namespace std;

#define VMBConvertPixelFormatString  "VMB_CONVERT_PIXEL_FORMAT"   // asynParamInt32, R/W
#define VMBTimeStampModeString       "VMB_TIME_STAMP_MODE"        // asynParamInt32, R/O
#define VMBUniqueIdModeString        "VMB_UNIQUE_ID_MODE"         // asynParamInt32, R/O

class ADVimbaFrameObserver : virtual public IFrameObserver {
public:
    ADVimbaFrameObserver(CameraPtr pCamera, class ADVimba *pVimba);
    ~ADVimbaFrameObserver();
    virtual void FrameReceived(const FramePtr pFrame);
    CameraPtr pCamera_;
    class ADVimba *pVimba_;  
};

/** Main driver class inherited from areaDetectors ADGenICam class.
 * One instance of this class will control one camera.
 */
class ADVimba : public ADGenICam
{
public:
    ADVimba(const char *portName, const char *cameraId,
            size_t maxMemory, int priority, int stackSize);

    // virtual methods to override from ADGenICam
    //virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value);
    //virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], 
                                size_t nElements, size_t *nIn);
    void report(FILE *fp, int details);
    virtual GenICamFeature *createFeature(GenICamFeatureSet *set, 
                                          std::string const & asynName, asynParamType asynType, int asynIndex,
                                          std::string const & featureName, GCFeatureType_t featureType);

    /**< These should be private but are called from C callback functions, must be public. */
    void imageGrabTask();
    void shutdown();
    CameraPtr getCamera();
    asynStatus processFrame(FramePtr pFrame);

private:
    inline asynStatus checkError(VmbErrorType error, const char *functionName, const char *message);
    int VMBConvertPixelFormat;
#define FIRST_VMB_PARAM VMBConvertPixelFormat;
    int VMBTimeStampMode;
    int VMBUniqueIdMode;

    /* Local methods to this class */
    asynStatus startCapture();
    asynStatus stopCapture();
    asynStatus connectCamera();
    asynStatus disconnectCamera();
    asynStatus readStatus();

    const char *cameraId_;
    CameraPtr pCamera_;
    VimbaSystem & system_;

    bool exiting_;
    bool acquiring_;
    epicsEventId startEventId_;
    epicsEventId newFrameEventId_;
    int uniqueId_;
    
    std::vector<string> statusFeatureNames_;

};

#endif

