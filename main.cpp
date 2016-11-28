#include "mainwindow.h"
#include <QApplication>
#include <QDebug>


#define M_PI       3.14159265358979323846
#define M_2PI      6.28318530717958647692

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  MainWindow w;
  w.show( );

  return a.exec( );
} // main
