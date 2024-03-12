// basic program for the ARDOS phantom. w. birkfellner, cmpbme, mu vienna, 2017

#include "mainwindow.h"
#include <QApplication>
#include <stdlib.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}

// standard routine in qt to start the GUI
