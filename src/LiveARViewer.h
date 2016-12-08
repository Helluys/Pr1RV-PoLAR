#ifndef LIVEARVIEWER_H
#define LIVEARVIEWER_H

#include <QObject>
#include <PoLAR/Viewer.h>

class LiveARViewer : public PoLAR::Viewer
{
    Q_OBJECT

    public:
        LiveARViewer(unsigned width, unsigned height, const osg::Matrix3d &K, float zNear = 0.0001, float zFar = 5000);
        ~LiveARViewer();
};

#endif // LIVEARVIEWER_H
