#ifndef LIVEARVIEWER_H
#define LIVEARVIEWER_H

#include <QObject>
#include <PoLAR/Viewer.h>

class LiveARViewer : public PoLAR::Viewer
{
    Q_OBJECT

    public:
        LiveARViewer(unsigned width, unsigned height);
        ~LiveARViewer();
};

#endif // LIVEARVIEWER_H
