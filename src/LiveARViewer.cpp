#include "LiveARViewer.h"


LiveARViewer::LiveARViewer(unsigned width, unsigned height, const osg::Matrix3d &K, float zFar, float zNear) : PoLAR::Viewer(nullptr, "LiveAR", 0, true)
{
    resize(width, height);
    osg::Matrixd P(   2*K(0, 0), -2*K(0, 1),   (width - 2*K(0, 2))/float(width) ,                             0,
                                 0,  2*K(1, 1), (-height + 2*K(1, 2))/float(height),                             0,
                                 0,          0,      -(zFar + zNear)/(zFar - zNear),  -2*zFar*zNear/(zFar - zNear));

    setProjection(P);
}


LiveARViewer::~LiveARViewer()
{
    //dtor
}
