#include <iostream>
#include "LiveARApp.h"

int main(int argc, char **argv)
{
    LiveARApp app(1024, 768, argc, argv);

    return app.exec();
}
