#include "LiveARApp.h"

LiveARApp::LiveARApp(unsigned width, unsigned height, int &argc, char **argv, unsigned camera) : mApp(argc, argv), mViewer(width, height), mTracker(mViewer), mCameraID(camera)
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
    mViewer.center();

    mObject3D = new PoLAR::FrameAxis;
    osg::Group *scene = mViewer.getScene3D();
    scene->addChild(mObject3D.get());

    mViewer.show();
    mApp.connect(&mApp, SIGNAL(lastWindowClosed()), &mApp, SLOT(quit()));
    QObject::connect(mCamera, SIGNAL(newFrame(unsigned char*,int,int,int)),
                        &mTracker, SLOT(newFrameReceived(unsigned char*,int,int,int)));

    return mApp.exec();
}

void LiveARApp::loadObject(const std::string &filename)
{
    mObject3D = new PoLAR::FrameAxis;
}
