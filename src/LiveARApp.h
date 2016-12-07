#ifndef LIVEARAPP_H
#define LIVEARAPP_H

#include <QtWidgets/QApplication>

#include <PoLAR/VideoPlayer.h>
#include <PoLAR/Image.h>
#include <osg/PositionAttitudeTransform>

#include "LiveARViewer.h"
#include "LiveARTracker.h"

class LiveARApp
{
    public:

        LiveARApp(unsigned width, unsigned height, int &argc, char **argv, unsigned camera = 0);
        ~LiveARApp();

        int exec();

    protected:
        QApplication mApp;

        LiveARViewer mViewer;
        LiveARTracker mTracker;

        int mCameraID;
        osg::ref_ptr<PoLAR::Object3D> mObject;
        osg::ref_ptr<PoLAR::VideoPlayer> mCamera;
        osg::ref_ptr<PoLAR::Image_uc> mImage, mReferenceImage;

};

#endif // LIVEARAPP_H
