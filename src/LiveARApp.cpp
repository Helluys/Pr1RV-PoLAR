#include "LiveARApp.h"

#include <PoLAR/FrameAxis.h>

LiveARApp::LiveARApp(unsigned width, unsigned height, int &argc, char **argv, unsigned camera) : mApp(argc, argv), mViewer(width, height), mTracker(nullptr), mCameraID(camera)
{
    mReferenceImage = new PoLAR::Image_uc("reference.png", true);
    mTracker.setReferenceImage(*mReferenceImage.get());

    mObject = new PoLAR::Object3D(new PoLAR::FrameAxis(), true, true, true);
}

LiveARApp::~LiveARApp()
{
    // smart pointers magic : no delete needed
}

int LiveARApp::exec()
{
    mCamera = new PoLAR::VideoPlayer(mCameraID);
    while(!mCamera->valid())
    {
        std::cerr << "Invalid camera, enter another camera ID (integer, -1 to quit) : ";
        std::cin >> mCameraID;
        if(mCameraID == -1) return 0;
        mCamera = new PoLAR::VideoPlayer(mCameraID);
    }

    mImage = new PoLAR::Image_uc(mCamera.get());
    mViewer.setBgImage(mImage.get());

    mCamera->play();

    mViewer.addObject3D(mObject);
    mTracker.setTransformedObject(mObject);

    mViewer.center();
    mViewer.show();

    QObject::connect(mCamera, SIGNAL(newFrame(unsigned char*,int,int,int)),
                     &mTracker, SLOT(newFrameReceived(unsigned char*,int,int,int)));

    mApp.connect(&mApp, SIGNAL(lastWindowClosed()), &mApp, SLOT(quit()));

    return mApp.exec();
}
