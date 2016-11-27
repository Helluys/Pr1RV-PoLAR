#include <iostream>
#include "LiveARApp.h"

int main(int argc, char **argv)
{
    LiveARApp app(800, 600, argc, argv);

    return app.exec();
}
