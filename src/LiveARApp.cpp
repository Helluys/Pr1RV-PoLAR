#include "LiveARApp.h"

LiveARApp::LiveARApp(unsigned width, unsigned height, int &argc, char **argv, unsigned camera) : mApp(argc, argv), mViewer(width, height), mTracker(nullptr), mCameraID(camera)
{
    mReferenceImage = new PoLAR::Image_uc("reference.png", true);
    mTracker.setReferenceImage(*mReferenceImage.get());
}

LiveARApp::~LiveARApp()
{

}

int LiveARApp::exec()
{
    mCamera = new PoLAR::VideoPlayer(mCameraID);
    while(!mCamera->valid())
    {
        std::cerr << "Invalid camera, enter another camera ID (integer, -1 to quit) : ";
        std::cin >> mCameraID;
        if(mCameraID == -1) return -1;
        mCamera = new PoLAR::VideoPlayer(mCameraID);
    }

    mImage = new PoLAR::Image_uc(mCamera.get());
    mViewer.setBgImage(mImage.get());

    mCamera->play();

    loadObject("armchair.obj");
    mViewer.addObject3D(mObject3D.get());
    mTracker.setTrackedObject(mObject3D);

    mViewer.center();
    mViewer.show();

    mApp.connect(&mApp, SIGNAL(lastWindowClosed()), &mApp, SLOT(quit()));
    QObject::connect(mCamera, SIGNAL(newFrame(unsigned char*,int,int,int)),
                        &mTracker, SLOT(newFrameReceived(unsigned char*,int,int,int)));

    return mApp.exec();
}

void LiveARApp::loadObject(const std::string &filename)
{
    mObject3D = new PoLAR::Object3D(filename, false, true);
    mObject3D->setName("object");
    mObject3D->optimize();
}
