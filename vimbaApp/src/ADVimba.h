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


class FrameObserver : virtual public IFrameObserver {
public:
    FrameObserver(CameraPtr pCamera, epicsMessageQueue *pMsgQ) 
  : IFrameObserver(pCamera),
     pMsgQ_(pMsgQ) 
    {}
    ~FrameObserver() {}
  
    virtual void FrameReceived(const FramePtr pFrame) {  
        if (pMsgQ_->send((void *)&pFrame, sizeof(pFrame)) != 0) {
            printf("FrameObserver::FrameReceived error calling pMsgQ_->send()\n");
        }
    }
private:
    epicsMessageQueue *pMsgQ_;
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

private:
    inline asynStatus checkError(VmbErrorType error, const char *functionName, const char *message);
    int VMBVideoMode;
#define FIRST_VMB_PARAM VMBVideoMode
    int VMBConvertPixelFormat;
    int VMBTransmitFailureCount;
    int VMBBufferUnderrunCount;
    int VMBFailedBufferCount;
    int VMBFailedPacketCount;
    int VMBTimeStampMode;
    int VMBUniqueIdMode;
    int VMBColorProcessEnabled;

    /* Local methods to this class */
    asynStatus grabImage();
    asynStatus startCapture();
    asynStatus stopCapture();
    asynStatus connectCamera();
    asynStatus disconnectCamera();
    asynStatus readStatus();

    const char *cameraId_;
    CameraPtr pCamera_;
    VimbaSystem & system_;
    FrameObserver *pFrameObserver_;

    int exiting_;
    epicsEventId startEventId_;
    epicsMessageQueue *pCallbackMsgQ_;
    NDArray *pRaw_;
    int uniqueId_;
};

#endif

