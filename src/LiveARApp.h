#ifndef LIVEARAPP_H
#define LIVEARAPP_H

#include <stdexcept>

#include <QtWidgets/QApplication>

#include <PoLAR/VideoPlayer.h>
#include <PoLAR/Image.h>

#include "LiveARViewer.h"

class LiveARException : public std::runtime_error
{
    public:
        enum Type
        {
            UNKNOWN,
            INVALID_CAMERA
        } type;

        LiveARException(const std::string &what = "", Type t = UNKNOWN) : std::runtime_error(what.c_str()), type(t)
        {}
};


class LiveARApp
{
    public:

        LiveARApp(unsigned width, unsigned height, int argc, char **argv, unsigned camera = 0);
        ~LiveARApp();

        int go();

        void setCamera(unsigned camera) {mCameraID = camera;}

    protected:
        QApplication mApp;

        LiveARViewer mViewer;

        osg::ref_ptr<PoLAR::VideoPlayer> mCamera;
        unsigned mCameraID;
        osg::ref_ptr<PoLAR::Image<unsigned char>> mImage;

};

#endif // LIVEARAPP_H
