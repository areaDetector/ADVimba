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
#include <iocsh.h>
#include <cantProceed.h>
#include <epicsString.h>
#include <epicsExit.h>

#include "VimbaCPP/Include/VimbaCPP.h"
#include "VimbaImageTransform/Include/VmbTransform.h"

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

// Size of message queue for callback function
#define CALLBACK_MESSAGE_QUEUE_SIZE 10

#define NUM_VIMBA_BUFFERS 10

typedef enum {
    VMBPixelConvertNone,
    VMBPixelConvertMono8,
    VMBPixelConvertMono16,
    VMBPixelConvertRGB8,
    VMBPixelConvertRGB16
} VMBPixelConvert_t;


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
} VMBTimeStamp_t;

typedef enum {
    UniqueIdCamera,
    UniqueIdDriver
} VMBUniqueId_t;


FrameObserver::FrameObserver(CameraPtr pCamera, class ADVimba *pVimba) 
    :   IFrameObserver(pCamera),
        pCamera_(pCamera), 
        pVimba_(pVimba)
{
}

FrameObserver::~FrameObserver() 
{
}
  
void FrameObserver::FrameReceived(const FramePtr pFrame) {
    pVimba_->processFrame(pFrame);
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
    cameraId_(cameraId),
    system_(VimbaSystem::GetInstance()), 
    exiting_(false),
    acquiring_(false),
    pRaw_(NULL), 
    uniqueId_(0)
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
 
    status = connectCamera();
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s:  camera connection failed (%d)\n",
            driverName, functionName, status);
        // Call report() to get a list of available cameras
        report(stdout, 1);
        return;
    }

    createParam("VMB_CONVERT_PIXEL_FORMAT",     asynParamInt32,   &VMBConvertPixelFormat);
    createParam("VMB_TIME_STAMP_MODE",          asynParamInt32,   &VMBTimeStampMode);
    createParam("VMB_UNIQUE_ID_MODE",           asynParamInt32,   &VMBUniqueIdMode);

    /* Set initial values of some parameters */
    setIntegerParam(NDDataType, NDUInt8);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);
    setIntegerParam(ADMinX, 0);
    setIntegerParam(ADMinY, 0);
    setStringParam(ADStringToServer, "<not used by driver>");
    setStringParam(ADStringFromServer, "<not used by driver>");
    
    pFrameObserver_ = new FrameObserver(pCamera_, this);

    startEventId_ = epicsEventCreate(epicsEventEmpty);
    newFrameEventId_ = epicsEventCreate(epicsEventEmpty);

    // launch image read task
    epicsThreadCreate("VimbaImageTask", 
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
    exiting_ = true;
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
    int acquire;
    int imageCounter;
    int numImages;
    int numImagesCounter;
    int imageMode;
    int arrayCallbacks;
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

        // We are now waiting for an image
        setIntegerParam(ADStatus, ADStatusWaiting);
        // Call the callbacks to update any changes
        callParamCallbacks();

        // Wait for event saying image has been collected
        unlock();
        epicsEventWait(newFrameEventId_);
        lock();
        getIntegerParam(NDArrayCounter, &imageCounter);
        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        getIntegerParam(ADImageMode, &imageMode);
        imageCounter++;
        numImagesCounter++;
        setIntegerParam(NDArrayCounter, imageCounter);
        setIntegerParam(ADNumImagesCounter, numImagesCounter);
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
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
            stopCapture();
        }
        callParamCallbacks();
    }
}

asynStatus ADVimba::processFrame(FramePtr pFrame)
{
    asynStatus status = asynSuccess;
    VmbError_t vmbStatus;
    VmbUint32_t nRows, nCols;
    NDDataType_t dataType;
    NDColorMode_t colorMode;
    VmbPixelFormatType pixelFormat;
    int convertPixelFormat;
    int numColors = 1;
    size_t dims[3];
    int pixelSize;
    size_t dataSize;
    int nDims;
    int uniqueIdMode;
    int timeStampMode;
    VmbUchar_t *pConvertBuffer = NULL;
    VmbUchar_t *pData = NULL;
    static const char *functionName = "processFrame";

    lock();
    VmbFrameStatusType frameStatus = VmbFrameStatusIncomplete;
    vmbStatus = pFrame->GetReceiveStatus(frameStatus);
    if (VmbErrorSuccess != vmbStatus || VmbFrameStatusComplete != frameStatus) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error GetReceiveStatus returned %d frameStatus=%d\n",
            driverName, functionName, vmbStatus, frameStatus);
        status = asynError;
        goto done;
    } 
    pFrame->GetWidth(nCols);
    pFrame->GetHeight(nRows);
    pFrame->GetPixelFormat(pixelFormat);
    pFrame->GetImage(pData);

    // Convert the pixel format if requested
    getIntegerParam(VMBConvertPixelFormat, &convertPixelFormat);
    if (convertPixelFormat != VMBPixelConvertNone) {
        VmbPixelFormatType outputPixelFormat;
//        VmbPixelLayout pixelLayout = VmbPixelLayoutMono;
        int bitsPerPixel = 8;
        switch (convertPixelFormat) {
            case VMBPixelConvertMono8:
//                pixelLayout = VmbPixelLayoutMono;
                outputPixelFormat = VmbPixelFormatMono8;
                bitsPerPixel = 8;
                numColors = 1;
                break;
            case VMBPixelConvertMono16:
//                pixelLayout = VmbPixelLayoutMono;
                outputPixelFormat = VmbPixelFormatMono16;
                bitsPerPixel = 16;
                numColors = 1;
                break;
            case VMBPixelConvertRGB8:
//                pixelLayout = VmbPixelLayoutRGB;
                outputPixelFormat = VmbPixelFormatRgb8;
                bitsPerPixel = 8;
                numColors = 3;
                break;
            case VMBPixelConvertRGB16:
//                pixelLayout = VmbPixelLayoutRGB;
                outputPixelFormat = VmbPixelFormatRgb16;
                bitsPerPixel = 16;
                numColors = 3;
                break;
            default:
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s Error: Unknown pixel conversion format %d\n",
                    driverName, functionName, convertPixelFormat);
                outputPixelFormat = VmbPixelFormatMono8;
                break;
        }
        VmbImage inputImage;
        VmbImage outputImage;
        VmbTransformInfo transformInfo;
        // Set size member for verification inside API
        inputImage.Size = sizeof(inputImage);
        outputImage.Size = sizeof(outputImage);
        // Attach the data buffers
        inputImage.Data = pData;
        pConvertBuffer = (VmbUchar_t *)malloc(nCols * nRows * numColors * bitsPerPixel/8);
        outputImage.Data = pConvertBuffer;
        // Fill image info from input pixel format
        vmbStatus = VmbSetImageInfoFromPixelFormat(pixelFormat, nCols, nRows, &inputImage);
        // Fill destination image info from output pixel format
        vmbStatus = VmbSetImageInfoFromPixelFormat(outputPixelFormat, nCols, nRows, &outputImage);
//        vmbStatus = VmbSetImageInfoFromInputImage(&inputImage, pixelLayout, bitsPerPixel, &outputImage);
        // Set the debayering algorithm to simple 2 by 2
        vmbStatus = VmbSetDebayerMode(VmbDebayerMode2x2, &transformInfo);
        // perform the transformation
        vmbStatus = VmbImageTransform(&inputImage, &outputImage, &transformInfo, 1);
        if (vmbStatus == VmbErrorSuccess) {
            pixelFormat = outputPixelFormat;
            pData = pConvertBuffer;
        } 
        else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
                "%s::%s error calling VmbImageTransform, input pixel format=0x%x, output pixel format=0x%x status return=%d\n", 
                driverName, functionName, pixelFormat, outputPixelFormat, vmbStatus);
        }
    }
    
    switch (pixelFormat) {
        case VmbPixelFormatMono8:
        case VmbPixelFormatBayerRG8:
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
            status = asynError;
            goto done;
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
        status = asynError;
        goto done;
    }
    if (pData) {
        memcpy(pRaw_->pData, pData, dataSize);
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s [%s] ERROR: pData is NULL!\n",
            driverName, functionName, portName);
        status = asynError;
        goto done;
    }

    // Put the frame number into the buffer
    getIntegerParam(VMBUniqueIdMode, &uniqueIdMode);
    if (uniqueIdMode == UniqueIdCamera) {
        VmbUint64_t uniqueId;
        pFrame->GetFrameID(uniqueId);
        pRaw_->uniqueId = uniqueId;
    } else {
        pRaw_->uniqueId = uniqueId_;
    }
    uniqueId_++;
    updateTimeStamp(&pRaw_->epicsTS);
    getIntegerParam(VMBTimeStampMode, &timeStampMode);
    // Set the timestamps in the buffer
    if (timeStampMode == TimeStampCamera) {
        VmbUint64_t timeStamp;
        pFrame->GetTimestamp(timeStamp);
        pRaw_->timeStamp = timeStamp / 1e9;
    } else {
        pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch + pRaw_->epicsTS.nsec/1e9;
    }

    // Get any attributes that have been defined for this driver        
    getAttributes(pRaw_->pAttributeList);
    pRaw_->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);
    
    callParamCallbacks();

    epicsEventSignal(newFrameEventId_);

    done:
    if (pConvertBuffer) free(pConvertBuffer);
    pCamera_->QueueFrame(pFrame);
    unlock();
    return status;
}

asynStatus ADVimba::readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], 
                               size_t nElements, size_t *nIn)
{
    int function = pasynUser->reason;
    //static const char *functionName = "readEnum";

    // There are a few enums we don't want to autogenerate the values
    if (function == VMBConvertPixelFormat) {
        return asynError;
    }
    
    return ADGenICam::readEnum(pasynUser, strings, values, severities, nElements, nIn);
}

asynStatus ADVimba::startCapture()
{
    //static const char *functionName = "startCapture";

    // If we are already acquiring return immediately
    if (acquiring_) return asynSuccess;

    // Start the camera transmission...
    setIntegerParam(ADNumImagesCounter, 0);
    setShutter(1);
    // How to we avoid having to create a new FrameObserver each time we start acquisition?
    // The smart pointer seems to be deleted by StartContinuousImageAcquisition?             
    pFrameObserver_ = new FrameObserver(pCamera_, this);
    pCamera_->StartContinuousImageAcquisition(NUM_VIMBA_BUFFERS, IFrameObserverPtr(pFrameObserver_));
    acquiring_ = true;
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
    pCamera_->StopContinuousImageAcquisition();
    acquiring_ = false;
    return asynSuccess;
}


asynStatus ADVimba::readStatus()
{
    std::vector<string> paramNames;
    GenICamFeature *pFeature;
    unsigned i;
    //static const char *functionName = "readStatus";
    
    paramNames.push_back("StatFrameDelivered");
    paramNames.push_back("StatFrameDropped");
    paramNames.push_back("StatFrameUnderrun");
    paramNames.push_back("StatPacketErrors");
    paramNames.push_back("StatPacketMissed");
    paramNames.push_back("StatPacketReceived");
    paramNames.push_back("StatPacketRequested");
    paramNames.push_back("StatPacketResent");
    paramNames.push_back("DeviceTemperature");

    for(i=0; i<paramNames.size(); i++) {
        pFeature = mGCFeatureSet.getByName(paramNames[i]);
        if (pFeature) pFeature->read(0, true);
    }
    if (acquiring_) return asynSuccess;
    return ADGenICam::readStatus();
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
