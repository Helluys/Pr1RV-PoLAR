#include <iostream>
#include "LiveARApp.h"

int main(int argc, char **argv)
{
    LiveARApp app(800, 600, argc, argv);

    bool run = false;
    int appReturn = 0;
    while(!run)
    {
        try
        {
            appReturn = app.go();
            run = true;
        }
        catch(LiveARException &e)
        {
            std::cerr << e.what() << std::endl;
            switch(e.type)
            {
                case LiveARException::UNKNOWN:
                    return -1;
                case LiveARException::INVALID_CAMERA:
                    app.setCamera(1);
                    break;
            }
        }
    }

    return appReturn;
}
