
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "VimbaCPP/Include/VimbaCPP.h"

using namespace AVT;
using namespace AVT::VmbAPI;
using namespace std;

class testVimba
{
public:
    testVimba(const char *cameraId);
    ~testVimba(void);
    void connectionCallback(UpdateTriggerType reason);
    int connectCamera();
    int disconnectCamera();
    CameraPtr pCamera_;

private:
    inline int checkError(VmbErrorType error, const char *functionName, const char *message);
    const char *cameraId_;
    VimbaSystem & system_;
    bool isConnected_;
};

class testCameraListObserver : public ICameraListObserver {
public:
    testCameraListObserver(class testVimba *pVimba)
        :   ICameraListObserver(),
            pVimba_(pVimba) {}
    ~testCameraListObserver() {};
    void CameraListChanged(CameraPtr pCam, UpdateTriggerType reason) {
        // We got a callback for a camera connect or disconnect event.
        // If it is for our camera then call testVimba::connectionCallback()
        CameraPtr myCamera = pVimba_->pCamera_;
        std::string serial, mySerial;
        myCamera->GetSerialNumber(mySerial);
        pCam->GetSerialNumber(serial);
        if (serial == mySerial) {
            pVimba_->connectionCallback(reason);
        }
    }
private:
    class testVimba *pVimba_;  
};

testVimba::testVimba(const char *cameraId)
    : 
    cameraId_(cameraId),
    system_(VimbaSystem::GetInstance()),
    isConnected_(false)
{
    if (checkError(system_.Startup(), "testVimba::testVimba", "VimbaSystem::Startup"))
        return;
    if (connectCamera())
        return;
    system_.RegisterCameraListObserver(ICameraListObserverPtr(new testCameraListObserver(this)));
    return;
}

testVimba::~testVimba(void)
{
    if (isConnected_) {
        disconnectCamera();
    }
    system_.Shutdown();
}

inline int testVimba::checkError(VmbErrorType error, const char *functionName, const char *VimbaFunction)
{
    if (VmbErrorSuccess != error) {
        printf("%s: ERROR calling %s error=%d\n", functionName, VimbaFunction, error);
        return -1;
    }
    return 0;
}

int testVimba::connectCamera(void)
{
    if (isConnected_) return 0;

    printf("testVimba::connectCamera calling OpenCameraByID with cameraID=%s\n", cameraId_);
    if (checkError(system_.OpenCameraByID(cameraId_, VmbAccessModeFull, pCamera_), "testVimba::connectCamera", "VimbaSystem::OpenCameraByID")) {
       return -1;
    }
    printf("testVimba::connectCamera connected to cameraID=%s\n", cameraId_);
    isConnected_ = true;
    return 0;
}

int testVimba::disconnectCamera(void)
{
    if (!isConnected_) return 0;
    if (checkError(pCamera_->Close(), "testVimba::disconnectCamera", "pCamera->Close()")) {
        return -1;
    }
    isConnected_ = false;
    printf("testVimba::disconnectCamera disconnect from cameraID=%s\n", cameraId_);
    return 0;
}
    
    
void testVimba::connectionCallback(UpdateTriggerType reason)
{
    printf("testVimba::connectionCallback,  reason=%d\n", reason);
    switch (reason) {
      case UpdateTriggerPluggedIn:
        connectCamera();
        break;
      case UpdateTriggerPluggedOut:
        disconnectCamera();
        break;
      case UpdateTriggerOpenStateChanged:
        break;
    }
}

int main(int argc, char *argv[])
{
  int sleepTime = 10;
  printf("main: creating testVimba object\n");
  testVimba *pV = new testVimba("164.54.160.57");
  printf("main: Sleeping for %d seconds\n", sleepTime);
  sleep(sleepTime);
  printf("main: calling disconnectCamera\n");
  pV->disconnectCamera();
  printf("main: Sleeping for %d seconds\n", sleepTime);
  sleep(sleepTime);
  printf("main: calling connectCamera()\n");
  pV->connectCamera();
  printf("main: Sleeping for %d seconds\n", sleepTime);
  sleep(sleepTime);
  printf("main: deleting testVimba object\n");
  delete pV;
}
