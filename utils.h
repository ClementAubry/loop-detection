#ifndef UTILS_H
#define UTILS_H

#include <QString>
#include <QTime>
#include <stdio.h>
#include <fstream>
#include<QColor>

#include "interval/box.h"
#include "interval/scalarTube.h"

// parse a string from the edit line to setup axes
Box parse(QString str, QString token)
{
    if (str.isEmpty())
        return Box(Interval(0,10),Interval(0,10));
    QStringList list = str.split(token);
    if (list.count() != 4)
        return Box(Interval(0,10),Interval(0,10));
    Box b(Interval(list.at(0).toDouble(), list.at(1).toDouble()),Interval(list.at(2).toDouble(), list.at(3).toDouble()));
    return b;
}
// parse a string from SLAM file data
Interval parseForSlam(QString str)
{
    if (str.isEmpty())
        return Interval(0,0);
    QStringList list = str.split(':');
    if (list.count() != 2)
      return Interval(0,0);
    str = list.at(1);
    //list.at(1) contient [x.xx,x.xx]
    str.remove('[');
    str.remove(']');
    list = str.split(',');
    if (list.count() != 2)
      return Interval(0,0);
    Interval i(list.at(0).toDouble(), list.at(1).toDouble());
    return i;
}
//Save datas
void printTubesToFile(const std::string & pFileName, ScalarTube & pTubeDx, ScalarTube & pTubeDy){
    QString myFileName(pFileName.c_str()+QTime::currentTime().toString()+QString(".txt"));

    std::ofstream fichier(myFileName.toStdString().c_str(), std::ios::out | std::ios::trunc);  // ouverture en écriture avec effacement du fichier ouvert

    if(fichier)
    {
        float dt = pTubeDx.dt;
        //On écrit l'entete
        fichier << "%Data from a simulation via matlab of AUVEN making a looping motion"<< std::endl;
        fichier << "% Sampling time"<< std::endl;
        fichier << dt<< std::endl;
        fichier << "%t phi dphi theta dtheta psi dpsi vx dvx vy dvy vz dvz depth ddepth a da x dx y dy"<< std::endl;
        //On écrit les données
        for (unsigned int i=0 ; i < pTubeDx.size() ; i++){
            fichier <<i*dt<<" 0 0 0 0 0 0 "<<pTubeDx.at(i).center()<<" "<<pTubeDx.at(i).width()/2<<" "<<pTubeDy.at(i).center()<<" "<<pTubeDy.at(i).width()/2<<"0 0 0 0 0 0 0 0 0 0"<< std::endl;
        }
        fichier.close();
    }
    else
        std::cerr << "Impossible d'ouvrir le fichier !" << std::endl;
}

QColor GetColour(double v,double vmin,double vmax)
{
   QColor c(255,255,255); // white
   double dv;

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      c.setRedF(0);
      c.setGreenF(4 * (v - vmin) / dv);
   } else if (v < (vmin + 0.5 * dv)) {
      c.setRedF(0);
      c.setBlueF(1 + 4 * (vmin + 0.25 * dv - v) / dv);
   } else if (v < (vmin + 0.75 * dv)) {
      c.setRedF(4 * (v - vmin - 0.5 * dv) / dv);
      c.setBlueF(0);
   } else {
      c.setGreenF(1 + 4 * (vmin + 0.75 * dv - v) / dv);
      c.setBlueF(0);
   }

   return(c);
}

//double interpolate( double val, double y0, double x0, double y1, double x1 ) {
//    return (val-x0)*(y1-y0)/(x1-x0) + y0;
//}

//double base( double val ) {
//    if ( val <= -0.75 ) return 0;
//    else if ( val <= -0.25 ) return interpolate( val, 0.0, -0.75, 1.0, -0.25 );
//    else if ( val <= 0.25 ) return 1.0;
//    else if ( val <= 0.75 ) return interpolate( val, 1.0, 0.25, 0.0, 0.75 );
//    else return 0.0;
//}

//double red( double gray ) {
//    return base( gray - 0.5 );
//}
//double green( double gray ) {
//    return base( gray );
//}
//double blue( double gray ) {
//    return base( gray + 0.5 );
//}

#endif // UTILS_H
