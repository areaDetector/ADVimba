/*=============================================================================
  Copyright (C) 2012 - 2016 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ListCameras.cpp

  Description: The ListCameras example will list all available cameras that
               are found by VimbaCPP.

-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

#include <sstream>
#include <iostream>
#include <vector>
#include <algorithm>

#include "ListCameras.h"

#include "VimbaCPP/Include/VimbaCPP.h"
#include "Common/StreamSystemInfo.h"
#include "Common/ErrorCodeToMessage.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

/**printing camera info for a camera.
*\note this function is used with for_each and is called for each camera in range cameras.begin(), cameraas.end()
*/
void PrintCameraInfo( const CameraPtr &camera )
{
    std::string strID;
    std::string strName;
    std::string strModelName;
    std::string strSerialNumber;
    std::string strInterfaceID;

    std::ostringstream ErrorStream;

    VmbErrorType err = camera->GetID( strID );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get camera ID. Error code: " << err << "("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "]";
        strID =  ErrorStream.str();
    }
                
    err = camera->GetName( strName );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get camera name. Error code: " << err << "("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "]";
        strName = ErrorStream.str() ;
    }

    err = camera->GetModel( strModelName );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get camera mode name. Error code: " << err << "("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "]";
        strModelName = ErrorStream.str();
    }

    err = camera->GetSerialNumber( strSerialNumber );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get camera serial number. Error code: " << err << "("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "]";
        strSerialNumber = ErrorStream.str();
    }

    err = camera->GetInterfaceID( strInterfaceID );
    if( VmbErrorSuccess != err )
    {
        ErrorStream << "[Could not get interface ID. Error code: " << err << "("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "]";
        strInterfaceID = ErrorStream.str() ;
    }

    std::cout   << "/// Camera Name    : " << strName           << "\n"
                << "/// Model Name     : " << strModelName      <<  "\n"
                << "/// Camera ID      : " << strID             <<  "\n"
                << "/// Serial Number  : " << strSerialNumber   <<  "\n"
                << "/// @ Interface ID : " << strInterfaceID    << "\n\n";
}

//
// Starts Vimba
// Gets all connected cameras
// And prints out information about the camera name, model name, serial number, ID and the corresponding interface ID
//
void ListCameras::Print()
{
    VimbaSystem&    sys = VimbaSystem::GetInstance();  // Get a reference to the VimbaSystem singleton
    std::cout<<"Vimba C++ API Version "<<sys<<"\n";           // Print out version of Vimba
    VmbErrorType    err = sys.Startup();               // Initialize the Vimba API
    CameraPtrVector cameras;                           // A vector of std::shared_ptr<AVT::VmbAPI::Camera> objects

    std::stringstream strError;

    if( VmbErrorSuccess == err )
    {
        err = sys.GetCameras( cameras );            // Fetch all cameras known to Vimba
        if( VmbErrorSuccess == err )
        {
            std::cout << "Cameras found: " << cameras.size() <<"\n\n";

            // Query all static details of all known cameras and print them out.
            // We don't have to open the cameras for that.
            std::for_each( cameras.begin(), cameras.end(), PrintCameraInfo );
        }
        else
        {
            std::cout << "Could not list cameras. Error code: " << err << "("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "\n";
        }

        sys.Shutdown();                             // Close Vimba
    }
    else
    {
        std::cout << "Could not start system. Error code: " << err <<"("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "\n";
    }
}

}}} // namespace AVT::VmbAPI::Examples

