/*
 * ADVimba.cpp
 *
 * This is a driver for AVT (Prosilica) cameras using their Vimba SDK
 *
 * Author: Mark Rivers
 *         University of Chicago
 *
 * Created: October 28, 2018
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <set>

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsMessageQueue.h>
#include <iocsh.h>
#include <cantProceed.h>
#include <epicsString.h>
#include <epicsExit.h>

#include "VimbaCPP/Include/VimbaCPP.h"

using namespace AVT;
using namespace AVT::VmbAPI;
using namespace std;

#include <ADGenICam.h>

#include <epicsExport.h>
#include "VimbaFeature.h"
#include "ADVimba.h"

#define DRIVER_VERSION      1
#define DRIVER_REVISION     0
#define DRIVER_MODIFICATION 0

static const char *driverName = "ADVimba";

/*
#define PGPacketSizeString            "PG_PACKET_SIZE"
#define PGPacketSizeActualString      "PG_PACKET_SIZE_ACTUAL"
#define PGMaxPacketSizeString         "PG_MAX_PACKET_SIZE"
#define PGPacketDelayString           "PG_PACKET_DELAY"
#define PGPacketDelayActualString     "PG_PACKET_DELAY_ACTUAL"
#define PGBandwidthString             "PG_BANDWIDTH"
*/

// Size of message queue for callback function
#define CALLBACK_MESSAGE_QUEUE_SIZE 10

#define NUM_VIMBA_BUFFERS 10

typedef enum {
    SPPixelConvertNone,
    SPPixelConvertMono8,
    SPPixelConvertMono16,
    SPPixelConvertRaw16,
    SPPixelConvertRGB8,
    SPPixelConvertRGB16
} SPPixelConvert_t;


/*
static const char *gigEPropertyTypeStrings[NUM_GIGE_PROPERTIES] = {
    "Heartbeat",
    "HeartbeatTimeout",
    "PacketSize",
    "PacketDelay"
};
*/

typedef enum {
    TimeStampCamera,
    TimeStampEPICS
} SPTimeStamp_t;

typedef enum {
    UniqueIdCamera,
    UniqueIdDriver
} SPUniqueId_t;


FrameObserver::FrameObserver(CameraPtr pCamera, epicsMessageQueue *pMsgQ) 
    :   IFrameObserver(pCamera),
        pMsgQ_(pMsgQ) 
{
}

FrameObserver::~FrameObserver() 
{
}
  
void FrameObserver::FrameReceived(const FramePtr pFrame) {
printf("FrameObserver::FrameReceived got frame, calling pMsgQ_->send()\n");
    if (pMsgQ_->send((void *)&pFrame, sizeof(pFrame)) != 0) {
        printf("FrameObserver::FrameReceived error calling pMsgQ_->send()\n");
    }
}

/** Configuration function to configure one camera.
 *
 * This function need to be called once for each camera to be used by the IOC. A call to this
 * function instanciates one object from the ADVimba class.
 * \param[in] portName asyn port name to assign to the camera.
 * \param[in] cameraId The camera index or serial number; <1000 is assumed to be index, >=1000 is assumed to be serial number.
 * \param[in] traceMask The initial value of the asynTraceMask.  
 *            If set to 0 or 1 then asynTraceMask will be set to ASYN_TRACE_ERROR.
 *            If set to 0x21 (ASYN_TRACE_WARNING | ASYN_TRACE_ERROR) then each call to the
 *            FlyCap2 library will be traced including during initialization.
 * \param[in] memoryChannel  The camera memory channel (non-volatile memory containing camera parameters) 
 *            to load during initialization.  If 0 no memory channel is loaded.
 *            If >=1 thenRestoreFromMemoryChannel(memoryChannel-1) is called.  
 *            Set memoryChannel to 1 to work around a bug in the Linux GigE driver in R2.0.
 * \param[in] maxMemory Maximum memory (in bytes) that this driver is allowed to allocate. So if max. size = 1024x768 (8bpp)
 *            and maxBuffers is, say 14. maxMemory = 1024x768x14 = 11010048 bytes (~11MB). 0=unlimited.
 * \param[in] priority The EPICS thread priority for this driver.  0=use asyn default.
 * \param[in] stackSize The size of the stack for the EPICS port thread. 0=use asyn default.
 */
extern "C" int ADVimbaConfig(const char *portName, const char *cameraId, 
                             size_t maxMemory, int priority, int stackSize)
{
    new ADVimba(portName, cameraId, maxMemory, priority, stackSize);
    return asynSuccess;
}


static void c_shutdown(void *arg)
{
   ADVimba *p = (ADVimba *)arg;
   p->shutdown();
}


static void imageGrabTaskC(void *drvPvt)
{
    ADVimba *pPvt = (ADVimba *)drvPvt;

    pPvt->imageGrabTask();
}

/** Constructor for the ADVimba class
 * \param[in] portName asyn port name to assign to the camera.
 * \param[in] cameraId The camera index or serial number; <1000 is assumed to be index, >=1000 is assumed to be serial number.
 * \param[in] traceMask The initial value of the asynTraceMask.  
 *            If set to 0 or 1 then asynTraceMask will be set to ASYN_TRACE_ERROR.
 *            If set to 0x21 (ASYN_TRACE_WARNING | ASYN_TRACE_ERROR) then each call to the
 *            FlyCap2 library will be traced including during initialization.
 * \param[in] memoryChannel  The camera memory channel (non-volatile memory containing camera parameters) 
 *            to load during initialization.  If 0 no memory channel is loaded.
 *            If >=1 thenRestoreFromMemoryChannel(memoryChannel-1) is called.  
 *            Set memoryChannel to 1 to work around a bug in the Linux GigE driver in R2.0.
 * \param[in] maxMemory Maximum memory (in bytes) that this driver is allowed to allocate. So if max. size = 1024x768 (8bpp)
 *            and maxBuffers is, say 14. maxMemory = 1024x768x14 = 11010048 bytes (~11MB). 0=unlimited.
 * \param[in] priority The EPICS thread priority for this driver.  0=use asyn default.
 * \param[in] stackSize The size of the stack for the EPICS port thread. 0=use asyn default.
 */
ADVimba::ADVimba(const char *portName, const char *cameraId,
                         size_t maxMemory, int priority, int stackSize )
    : ADGenICam(portName, maxMemory, priority, stackSize),
    cameraId_(cameraId), system_(VimbaSystem::GetInstance()), exiting_(0), pRaw_(NULL), uniqueId_(0)
{
    static const char *functionName = "ADVimba";
    asynStatus status;
    char tempString[100];
    VmbVersionInfo_t version;

    epicsSnprintf(tempString, sizeof(tempString), "%d.%d.%d", 
                  DRIVER_VERSION, DRIVER_REVISION, DRIVER_MODIFICATION);
    setStringParam(NDDriverVersion,tempString);
    
    checkError(system_.Startup(), functionName, "VimbaSystem::Startup");
    checkError(system_.QueryVersion(version), functionName, "VimbaSystem::QueryVersion");
    epicsSnprintf(tempString, sizeof(tempString), "%d.%d.%d", 
                  version.major, version.minor, version.patch);
    setStringParam(ADSDKVersion,tempString);
 

/*
    createParam(PGPacketSizeString,             asynParamInt32,   &PGPacketSize);
    createParam(PGPacketSizeActualString,       asynParamInt32,   &PGPacketSizeActual);
    createParam(PGMaxPacketSizeString,          asynParamInt32,   &PGMaxPacketSize);
    createParam(PGPacketDelayString,            asynParamInt32,   &PGPacketDelay);
    createParam(PGPacketDelayActualString,      asynParamInt32,   &PGPacketDelayActual);
    createParam(PGBandwidthString,              asynParamFloat64, &PGBandwidth);
*/

    status = connectCamera();
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s:  camera connection failed (%d)\n",
            driverName, functionName, status);
        // Call report() to get a list of available cameras
        report(stdout, 1);
        return;
    }

    //createParam("SP_CONVERT_PIXEL_FORMAT",     asynParamInt32,   &SPConvertPixelFormat);

    //createParam("SP_BUFFER_UNDERRUN_COUNT",    asynParamInt32,   &SPBufferUnderrunCount);
    //createParam("SP_FAILED_BUFFER_COUNT",      asynParamInt32,   &SPFailedBufferCount);
    //createParam("SP_FAILED_PACKET_COUNT",      asynParamInt32,   &SPFailedPacketCount);
    //createParam("SP_TIME_STAMP_MODE",          asynParamInt32,   &SPTimeStampMode);
    //createParam("SP_UNIQUE_ID_MODE",           asynParamInt32,   &SPUniqueIdMode);

    /* Set initial values of some parameters */
    setIntegerParam(NDDataType, NDUInt8);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);
    setIntegerParam(ADMinX, 0);
    setIntegerParam(ADMinY, 0);
    setStringParam(ADStringToServer, "<not used by driver>");
    setStringParam(ADStringFromServer, "<not used by driver>");
//    setIntegerParam(SPTriggerSource, 0);
//    setSPProperty(SPColorProcessEnabled, 0);
    
//    getSPProperty(ADMaxSizeX, &iValue);
//    setIntegerParam(ADSizeX, iValue);
//    getSPProperty(ADMaxSizeY, &iValue);
//    setIntegerParam(ADSizeY, iValue);

    // Create the message queue to pass images from the callback class
    pCallbackMsgQ_ = new epicsMessageQueue(CALLBACK_MESSAGE_QUEUE_SIZE, sizeof(FramePtr));
    if (!pCallbackMsgQ_) {
        cantProceed("ADVimba::ADVimba epicsMessageQueueCreate failure\n");
    }

    pFrameObserver_ = new FrameObserver(pCamera_, pCallbackMsgQ_);

    startEventId_ = epicsEventCreate(epicsEventEmpty);

    // launch image read task
    epicsThreadCreate("PointGreyImageTask", 
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      imageGrabTaskC, this);

    // shutdown on exit
    epicsAtExit(c_shutdown, this);

    return;
}

inline asynStatus ADVimba::checkError(VmbErrorType error, const char *functionName, const char *PGRFunction)
{
    if (VmbErrorSuccess != error) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: ERROR calling %s error=%d\n",
            driverName, functionName, PGRFunction, error);
        return asynError;
    }
    return asynSuccess;
}


void ADVimba::shutdown(void)
{
    //static const char *functionName = "shutdown";
    
    lock();
    exiting_ = 1;
    delete pFrameObserver_;
    pCamera_->Close();
    system_.Shutdown();
    unlock();
}

GenICamFeature *ADVimba::createFeature(GenICamFeatureSet *set, 
                                       std::string const & asynName, asynParamType asynType, int asynIndex,
                                       std::string const & featureName, GCFeatureType_t featureType) {
    return new VimbaFeature(set, asynName, asynType, asynIndex, featureName, featureType, pCamera_);
}

asynStatus ADVimba::connectCamera(void)
{
    static const char *functionName = "connectCamera";

    if (checkError(system_.OpenCameraByID(cameraId_, VmbAccessModeFull, pCamera_), functionName, 
                   "VimbaSystem::OpenCameraByID")) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s error opening camera %s\n", driverName, functionName, cameraId_);
    }
    return asynSuccess;
}


/** Task to grab images off the camera and send them up to areaDetector
 *
 */

void ADVimba::imageGrabTask()
{
    asynStatus status = asynSuccess;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int arrayCallbacks;
    epicsTimeStamp startTime;
    int acquire;
    static const char *functionName = "imageGrabTask";

    lock();

    while (1) {
        // Is acquisition active? 
        getIntegerParam(ADAcquire, &acquire);
        // If we are not acquiring then wait for a semaphore that is given when acquisition is started 
        if (!acquire) {
            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();

            // Wait for a signal that tells this thread that the transmission
            // has started and we can start asking for image buffers...
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s waiting for acquire to start\n", 
                driverName, functionName);
            // Release the lock while we wait for an event that says acquire has started, then lock again
            unlock();
            epicsEventWait(startEventId_);
            lock();
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s started!\n", 
                driverName, functionName);
            setIntegerParam(ADNumImagesCounter, 0);
            setIntegerParam(ADAcquire, 1);
        }

        // Get the current time 
        epicsTimeGetCurrent(&startTime);
        // We are now waiting for an image
        setIntegerParam(ADStatus, ADStatusWaiting);
        // Call the callbacks to update any changes
        callParamCallbacks();

        status = grabImage();
        if (status == asynError) {
            // remember to release the NDArray back to the pool now
            // that we are not using it (we didn't get an image...)
            if (pRaw_) pRaw_->release();
            pRaw_ = NULL;
            continue;
        }

        getIntegerParam(NDArrayCounter, &imageCounter);
        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
        imageCounter++;
        numImagesCounter++;
        setIntegerParam(NDArrayCounter, imageCounter);
        setIntegerParam(ADNumImagesCounter, numImagesCounter);

        if (arrayCallbacks) {
            // Call the NDArray callback
            doCallbacksGenericPointer(pRaw_, NDArrayData, 0);
        }
        // Release the NDArray buffer now that we are done with it.
        // After the callback just above we don't need it anymore
        pRaw_->release();
        pRaw_ = NULL;

        // See if acquisition is done if we are in single or multiple mode
        if ((imageMode == ADImageSingle) || ((imageMode == ADImageMultiple) && (numImagesCounter >= numImages))) {
            setIntegerParam(ADStatus, ADStatusIdle);
            status = stopCapture();
        }
        callParamCallbacks();
    }
}

asynStatus ADVimba::grabImage()
{
    asynStatus status = asynSuccess;
    VmbUint32_t nRows, nCols;
    NDDataType_t dataType;
    NDColorMode_t colorMode;
    VmbPixelFormatType pixelFormat;
    //int convertPixelFormat;
    int numColors;
    size_t dims[3];
    int pixelSize;
    size_t dataSize;
    int nDims;
    FramePtr pFrame;
    static const char *functionName = "grabImage";

    while(1) {
        unlock();
printf("%s::%s waiting for frame\n", driverName, functionName);
        int recvSize = pCallbackMsgQ_->receive(&pFrame, sizeof(pFrame), 0.1);
        lock();
        if (recvSize == sizeof(pFrame)) {
            break;
        } else if (recvSize == -1) {
            // Timeout
            int acquire;
            getIntegerParam(ADAcquire, &acquire);
            if (acquire == 0) {
                return asynError;
            } else {
                continue;
            }
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s error receiving from message queue\n",
                    driverName, functionName);
            return asynError;
        }
    }
printf("%s::%s got frame\n", driverName, functionName);
    VmbFrameStatusType frameStatus = VmbFrameStatusIncomplete;
    VmbErrorType receiveStatus;
    receiveStatus = pFrame->GetReceiveStatus(frameStatus);
    if (VmbErrorSuccess != receiveStatus || VmbFrameStatusComplete != frameStatus) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error GetReceiveStatus returned %d frameStatus=%d\n",
            driverName, functionName, receiveStatus, frameStatus);
        pCamera_->QueueFrame(pFrame);
        return asynError;
    } 
    pFrame->GetWidth(nCols);
    pFrame->GetHeight(nRows);

/*     
    // Convert the pixel format if requested
    getIntegerParam(SPConvertPixelFormat, &convertPixelFormat);
    if (convertPixelFormat != SPPixelConvertNone) {
        PixelFormatEnums convertedFormat;
        switch (convertPixelFormat) {
            case SPPixelConvertMono8:
                convertedFormat = PixelFormat_Mono8;
                break;
            case SPPixelConvertMono16:
                convertedFormat = PixelFormat_Mono16;
                break;
            case SPPixelConvertRaw16:
                convertedFormat = PixelFormat_Raw16;
                break;
            case SPPixelConvertRGB8:
                convertedFormat = PixelFormat_RGB8;
                break;
            case SPPixelConvertRGB16:
                convertedFormat = PixelFormat_RGB16;
                break;
            default:
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s Error: Unknown pixel conversion format %d\n",
                    driverName, functionName, convertPixelFormat);
                convertedFormat = PixelFormat_Mono8;
                break;
        }

        pixelFormat = pImage->GetPixelFormat();
printf("Converting image from format 0x%x to format 0x%x\n", pixelFormat, convertedFormat);
        try {
            ImagePtr pConvertedImage = pImage->Convert(convertedFormat);
            pImage->Release();
            pImage = pConvertedImage;
        }
        catch (Spinnaker::Exception &e) {
             asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
                 "%s::%s exception %s\n",
             driverName, functionName, e.what());
        }
    }
*/    
    pFrame->GetPixelFormat(pixelFormat);
    switch (pixelFormat) {
        case VmbPixelFormatMono8:
            dataType = NDUInt8;
            colorMode = NDColorModeMono;
            numColors = 1;
            pixelSize = 1;
            break;

        case VmbPixelFormatRgb8:
            dataType = NDUInt8;
            colorMode = NDColorModeRGB1;
            numColors = 3;
            pixelSize = 1;
            break;

        case VmbPixelFormatMono16:
            dataType = NDUInt16;
            colorMode = NDColorModeMono;
            numColors = 1;
            pixelSize = 2;
            break;

        case VmbPixelFormatRgb16:
            dataType = NDUInt16;
            colorMode = NDColorModeRGB1;
            numColors = 3;
            pixelSize = 2;
            break;

        default:
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unsupported pixel format=0x%x\n",
                driverName, functionName, pixelFormat);
            return asynError;
    }

    if (numColors == 1) {
        nDims = 2;
        dims[0] = nCols;
        dims[1] = nRows;
    } else {
        nDims = 3;
        dims[0] = 3;
        dims[1] = nCols;
        dims[2] = nRows;
    }
    dataSize = dims[0] * dims[1] * pixelSize;
    if (nDims == 3) dataSize *= dims[2];
    VmbUint32_t vmbDataSize;
    pFrame->GetImageSize(vmbDataSize);
    if (dataSize != vmbDataSize) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: data size mismatch: calculated=%lu, reported=%u\n",
            driverName, functionName, (long)dataSize, vmbDataSize);
        //return asynError;
    }
    setIntegerParam(NDArraySizeX, (int)nCols);
    setIntegerParam(NDArraySizeY, (int)nRows);
    setIntegerParam(NDArraySize, (int)dataSize);
    setIntegerParam(NDDataType,dataType);
    if (nDims == 3) {
        colorMode = NDColorModeRGB1;
    } else {
        // If the color mode is currently set to Bayer leave it alone
        getIntegerParam(NDColorMode, (int *)&colorMode);
        if (colorMode != NDColorModeBayer) colorMode = NDColorModeMono;
    }
    setIntegerParam(NDColorMode, colorMode);

    pRaw_ = pNDArrayPool->alloc(nDims, dims, dataType, 0, NULL);
    if (!pRaw_) {
        // If we didn't get a valid buffer from the NDArrayPool we must abort
        // the acquisition as we have nowhere to dump the data...
        setIntegerParam(ADStatus, ADStatusAborting);
        callParamCallbacks();
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s [%s] ERROR: Serious problem: not enough buffers left! Aborting acquisition!\n",
            driverName, functionName, portName);
        setIntegerParam(ADAcquire, 0);
        return(asynError);
    }
    VmbUchar_t *pData;
    pFrame->GetImage(pData);
    if (pData) {
        memcpy(pRaw_->pData, pData, dataSize);
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s [%s] ERROR: pData is NULL!\n",
            driverName, functionName, portName);
        return asynError;
    }

    // Put the frame number into the buffer
//    getIntegerParam(SPUniqueIdMode, &uniqueIdMode);
    int uniqueIdMode = UniqueIdCamera;
    if (uniqueIdMode == UniqueIdCamera) {
        VmbUint64_t uniqueId;
        pFrame->GetFrameID(uniqueId);
        pRaw_->uniqueId = uniqueId;
    } else {
        pRaw_->uniqueId = uniqueId_;
    }
    uniqueId_++;
    updateTimeStamp(&pRaw_->epicsTS);
//    getIntegerParam(SPTimeStampMode, &timeStampMode);
    // Set the timestamps in the buffer
    int timeStampMode = TimeStampCamera;
    if (timeStampMode == TimeStampCamera) {
        VmbUint64_t timeStamp;
        pFrame->GetTimestamp(timeStamp);
        pRaw_->timeStamp = timeStamp / 1e9;
    } else {
        pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch + pRaw_->epicsTS.nsec/1e9;
    }
    pCamera_->QueueFrame(pFrame);

    // Get any attributes that have been defined for this driver        
    getAttributes(pRaw_->pAttributeList);
    
    // Change the status to be readout...
    setIntegerParam(ADStatus, ADStatusReadout);
    callParamCallbacks();

    pRaw_->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);
    return status;

}

asynStatus ADVimba::readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], 
                               size_t nElements, size_t *nIn)
{
//    int function = pasynUser->reason;
    //static const char *functionName = "readEnum";

    // There are a few enums we don't want to autogenerate the values
//    if (function == SPConvertPixelFormat) {
//        return asynError;
//    }
    
    return ADGenICam::readEnum(pasynUser, strings, values, severities, nElements, nIn);
}

asynStatus ADVimba::startCapture()
{
    //static const char *functionName = "startCapture";
    
    // Start the camera transmission...
    setIntegerParam(ADNumImagesCounter, 0);
    setShutter(1);
printf("Calling StartContinuousImageAcquisition\n");
    pCamera_->StartContinuousImageAcquisition(NUM_VIMBA_BUFFERS, IFrameObserverPtr(pFrameObserver_));
    epicsEventSignal(startEventId_);
    return asynSuccess;
}


asynStatus ADVimba::stopCapture()
{
    int status;
    //static const char *functionName = "stopCapture";

    setIntegerParam(ADAcquire, 0);
    setShutter(0);
    // Need to wait for the task to set the status to idle
    while (1) {
        getIntegerParam(ADStatus, &status);
        if (status == ADStatusIdle) break;
        unlock();
        epicsThreadSleep(.1);
        lock();
    }
    //pCamera_->StopContinuousImageAcquisition();
    FramePtr pFrame;
    // Need to empty the message queue it could have some images in it
    while(pCallbackMsgQ_->tryReceive(&pFrame, sizeof(pFrame)) != -1) {}
    return asynSuccess;
}


asynStatus ADVimba::readStatus()
{
/*
    static const char *functionName = "readStatus";

    const TransportLayerStream& camInfo = pCamera_->TLStream;
	  cout << "Stream ID: " << camInfo.StreamID.ToString() << endl;
	  cout << "Stream Type: " << camInfo.StreamType.ToString() << endl;
    cout << "Stream Buffer Count: " << camInfo.StreamDefaultBufferCount.ToString() << endl;
    cout << "Stream Buffer Handling Mode: " << camInfo.StreamBufferHandlingMode.ToString() << endl;
    cout << "Stream Packets Received: " << camInfo.GevTotalPacketCount.ToString() << endl;
    getSPProperty(ADTemperatureActual);
    setIntegerParam(SPBufferUnderrunCount, (int)camInfo.StreamBufferUnderrunCount.GetValue());
    setIntegerParam(SPFailedBufferCount,   (int)camInfo.StreamFailedBufferCount.GetValue());
    if (camInfo.StreamType.GetIntValue() == StreamType_GEV) {
        setIntegerParam(SPFailedPacketCount,   (int)camInfo.GevFailedPacketCount.GetValue());
    }
*/
    callParamCallbacks();
    return asynSuccess;

}

void ADVimba::report(FILE *fp, int details)
{
    int numCameras;
    int i;
    static const char *functionName = "report";

    CameraPtrVector cameras;
    checkError(system_.GetCameras(cameras), functionName, "VimbaSystem::GetCameras()");
    numCameras = cameras.size();
    fprintf(fp, "\nNumber of cameras detected: %d\n", numCameras);
    if (details > 1) {
        CameraPtr pCamera;
        for (i=0; i<numCameras; i++) {
            pCamera = cameras[i];
            fprintf(fp, "Camera %d\n", i);
            std::string str;
            pCamera->GetName(str);
            fprintf(fp, "            Name: %s\n", str.c_str());
            pCamera->GetModel(str);
            fprintf(fp, "           Model: %s\n", str.c_str());
            pCamera->GetSerialNumber(str);
            fprintf(fp, "        Serial #: %s\n", str.c_str());
            pCamera->GetInterfaceID(str);
            fprintf(fp, "    Interface ID: %s\n", str.c_str());
            //pCamera->GetInterfaceType(str);
            //fprintf(fp, "  Interface type: %s\n", str.c_str());
        }
    }
    
    ADGenICam::report(fp, details);
    return;
}


static const iocshArg configArg0 = {"Port name", iocshArgString};
static const iocshArg configArg1 = {"cameraId", iocshArgString};
static const iocshArg configArg2 = {"maxMemory", iocshArgInt};
static const iocshArg configArg3 = {"priority", iocshArgInt};
static const iocshArg configArg4 = {"stackSize", iocshArgInt};
static const iocshArg * const configArgs[] = {&configArg0,
                                              &configArg1,
                                              &configArg2,
                                              &configArg3,
                                              &configArg4};
static const iocshFuncDef configADVimba = {"ADVimbaConfig", 5, configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
    ADVimbaConfig(args[0].sval, args[1].sval, args[2].ival, 
                  args[3].ival, args[4].ival);
}


static void ADVimbaRegister(void)
{
    iocshRegister(&configADVimba, configCallFunc);
}

extern "C" {
epicsExportRegistrar(ADVimbaRegister);
}
