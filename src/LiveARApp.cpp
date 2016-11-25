#include "LiveARApp.h"

LiveARApp::LiveARApp(unsigned width, unsigned height, int argc, char **argv, unsigned camera) : mApp(argc, argv), mViewer(width, height), mCameraID(camera)
{
}

LiveARApp::~LiveARApp()
{}

int LiveARApp::go()
{
    mCamera = new PoLAR::VideoPlayer(mCameraID);
    if(!mCamera.get())
        throw LiveARException("Invalid camera ID", LiveARException::INVALID_CAMERA);

    mImage = new PoLAR::Image_uc(mCamera.get());
    mViewer.setBgImage(mImage.get());

    mCamera->play();
    mViewer.center();
    mViewer.show();

    mApp.connect(&mApp, SIGNAL(lastWindowClosed()), &mApp, SLOT(quit()));
    return mApp.exec();
}
