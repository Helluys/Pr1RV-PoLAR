#ifndef LIVEARAPP_H
#define LIVEARAPP_H

#include <QtWidgets/QApplication>

#include <PoLAR/VideoPlayer.h>
#include <PoLAR/Image.h>
#include <PoLAR/FrameAxis.h>

#include "LiveARViewer.h"
#include "LiveARTracker.h"

class LiveARApp
{
    public:

        LiveARApp(unsigned width, unsigned height, int &argc, char **argv, unsigned camera = 0);
        ~LiveARApp();

        int exec();

        void loadObject(const std::string &filename);

    protected:
        QApplication mApp;

        LiveARViewer mViewer;
        LiveARTracker mTracker;

        int mCameraID;
        osg::ref_ptr<PoLAR::VideoPlayer> mCamera;
        osg::ref_ptr<PoLAR::Image_uc> mImage, mReferenceImage;

        osg::ref_ptr<PoLAR::FrameAxis> mObject3D;

};

#endif // LIVEARAPP_H
