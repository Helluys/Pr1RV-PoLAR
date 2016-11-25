#include "LiveARViewer.h"


LiveARViewer::LiveARViewer(unsigned width, unsigned height) : PoLAR::Viewer(nullptr, "LiveAR", 0, true)
{
    resize(width, height);
}


LiveARViewer::~LiveARViewer()
{
    //dtor
}
