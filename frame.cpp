#include "frame.h"
#include <cmath>
#include <stdio.h>
#include <algorithm>    //for min_element
#include <functional>   // for less
#include <QDebug>
#include <QMessageBox>
#include <QTime>
#include <QPicture>

#define sgn(a) ( (a) < 0 )
using namespace std;



Frame::Frame(QLabel * abscMinLbl, QLabel * abscMeanLbl, QLabel * abscMaxLbl, QLabel * ordMinLbl, QLabel * ordMeanLbl, QLabel * ordMaxLbl, Box & b, bool pDrawRect, QWidget *parent, QPen pPen)
  : QWidget(parent), pAbsMinLbl(abscMinLbl), pAbsMeanLbl(abscMeanLbl), pAbsMaxLbl(abscMaxLbl),
    pOrdMinLbl(ordMinLbl), pOrdMeanLbl(ordMeanLbl), pOrdMaxLbl(ordMaxLbl)
{
  setBackgroundRole(QPalette::Base);
  setAutoFillBackground(true);
  echx=0.0;
  echy=0.0;
  lPen = pPen;
  drawClickedBox = false;
  drawZeroLine = false;
  plotGrayScale = false;
  drawRect = pDrawRect;
  setPriorDomain(b);
  initDom = b;
}
void Frame::setPriorDomain(Box &b)
{
  priorDomain = b;
  echx = width() /priorDomain[1].width();
  echy = height()/priorDomain[2].width();
  //Mise a jour des labels des axes
  char buf[100];
  sprintf(buf,"%f\r",b[1].inf);
  pAbsMinLbl->setText(buf);
  buf[0] = '\r';
  sprintf(buf,"%.2f\r",b[1].center());
  pAbsMeanLbl->setText(buf);
  buf[0] = '\r';
  sprintf(buf,"%.2f\r",b[1].sup);
  pAbsMaxLbl->setText(buf);
  buf[0] = '\r';
  sprintf(buf,"%.2f\r",b[2].inf);
  pOrdMinLbl->setText(buf);
  buf[0] = '\r';
  sprintf(buf,"%.2f\r",b[2].center());
  pOrdMeanLbl->setText(buf);
  buf[0] = '\r';
  sprintf(buf,"%.2f\r",b[2].sup);
  pOrdMaxLbl->setText(buf);
  update();
}
void Frame::resizeEvent(QResizeEvent *)
{
  echx = width() /priorDomain[1].width();
  echy = height()/priorDomain[2].width();
}
void Frame::setInitialDomain(Box & b)
{
  initDom = b;
  setPriorDomain(b);
}
double Frame::xToPix(double x)
{
  return (x-priorDomain[1].inf)*echx;
}
double Frame::yToPix(double y)
{
  return height()-(y-priorDomain[2].inf)*echy;
}
double Frame::pixToX(double xPos)
{
  return priorDomain[1].inf + (xPos/echx);
}
double Frame::pixToY(double yPos)
{
  return priorDomain[2].inf -(yPos-height())/echy;
}
void Frame::setBoxAtT1(Box& b,QColor c)
{
  boxAtT1.box = b;
  boxAtT1.color = c;
  boxAtT1.color.setAlpha(127);
}
void Frame::setBoxAtT2(Box& b,QColor c)
{
  boxAtT2.box = b;
  boxAtT2.color = c;
  boxAtT2.color.setAlpha(127);
}
void Frame::clear()
{
  drawClickedBox = false;
  drawZeroLine = false;
  tubeList.clear();
  boxList.clear();
  pointList.clear();
  boldPointList.clear();
  kalmanPointList.clear();
  newtonList.clear();
  update();
}
void Frame::addTube(ScalarTube& t,int startTime,  QColor c)
{
  coloredTube ct;
  ct.startTime = startTime;
  ct.tube = t;
  ct.color = c;
  tubeList.push_front(ct);
  updateAxisToData();
}
void Frame::addBox(unsigned int t, Box &b, QColor c)
{
  coloredRect cr;
  cr.box = b;
  cr.color = c;
  cr.color.setAlpha(128);
  boxList[t] = cr;
}
void Frame::addNewtonBox(Box &b, QColor c)
{
  coloredRect cr;
  cr.box = b;
  cr.color = c;
  newtonList.push_front(cr);
}
void Frame::plotNewtonTestedBox(Box &b, QColor c)
{
  coloredRect cr;
  cr.box = b;
  cr.color = c;
  newtonTestedBox = cr;
}
void Frame::plotNewtonContractedBox(Box &b, QColor c)
{
  coloredRect cr;
  cr.box = b;
  cr.color = c;
  newtonContractedBox = cr;
}
void Frame::addBoldPoint(QPointF& p,QColor c)
{
  coloredPoint cp;
  cp.point = p;
  cp.color = c;
  cp.width = 1;
  boldPointList.push_front(cp);
}
void Frame::addBoldPoint(double x, double y, QColor c)
{
  coloredPoint cp;
  cp.point.setX(x);
  cp.point.setY(y);
  cp.color = c;
  cp.width = 1;
  boldPointList.push_front(cp);
}
void Frame::addKalmanPoint(double x, double y, QColor c)
{
  coloredPoint cp;
  cp.point.setX(x);
  cp.point.setY(y);
  cp.color = c;
  //    cp.color.setAlpha(128);
  cp.width = 5;
  kalmanPointList.push_front(cp);
}
void Frame::addPoint(unsigned int t, QPointF& p,QColor c)
{
  coloredPoint cp;
  cp.point = p;
  cp.color = c;
  cp.width = 1;
  pointList[t]=cp;
}
void Frame::addPoint(unsigned int t, double x, double y, QColor c)
{
  coloredPoint cp;
  cp.point.setX(x);
  cp.point.setY(y);
  cp.color = c;
  cp.width = 1;
  pointList[t]=cp;
}
void Frame::addSecondPoint(unsigned int t, double x, double y, QColor c)
{
  coloredPoint cp;
  cp.point.setX(x);
  cp.point.setY(y);
  cp.color = c;
  cp.width = 1;
  secondPointList[t]=cp;
}
void Frame::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  //Config for painter
  painter.setPen(lPen);
  painter.setBrush(Qt::NoBrush);
  //Draw kalman points (drawn with transparency)
  QList<coloredPoint>::iterator lp;
  if (drawKalmanResults)
    {
      for(lp=this->kalmanPointList.begin(); lp != this->kalmanPointList.end(); ++lp){
          QPen myBoldPen(lp->color);
          myBoldPen.setWidth(lp->width);
          painter.setPen(myBoldPen);
          painter.drawPoint(QPoint(xToPix(lp->point.x()),yToPix(lp->point.y())));
        }
      painter.setPen(lPen);
    }
  //Draw Boxes
  if (drawLDResults)
    {
      QMap<unsigned int,coloredRect>::iterator li;
      QRect r;
      for(li=this->boxList.begin(); li != this->boxList.end(); ++li)
        {
          r.setTopLeft(QPoint(xToPix(li.value().box[1].inf),yToPix(li.value().box[2].sup)));
          r.setBottomRight(QPoint(xToPix(li.value().box[1].sup),yToPix(li.value().box[2].inf)));
          painter.fillRect(r,li.value().color);
          if (drawRect){
              painter.drawRect(r);
            }
        }
    }
  if (drawClickedBox){
      QRect rt1,rt2;
      rt1.setTopLeft(QPoint(xToPix(boxAtT1.box[1].inf),yToPix(boxAtT1.box[2].sup)));
      rt1.setBottomRight(QPoint(xToPix(boxAtT1.box[1].sup),yToPix(boxAtT1.box[2].inf)));
      rt2.setTopLeft(QPoint(xToPix(boxAtT2.box[1].inf),yToPix(boxAtT2.box[2].sup)));
      rt2.setBottomRight(QPoint(xToPix(boxAtT2.box[1].sup),yToPix(boxAtT2.box[2].inf)));
      painter.fillRect(rt1,boxAtT1.color);
      painter.fillRect(rt2,boxAtT2.color);
      if (drawRect){
          painter.drawRect(rt1);
          painter.drawRect(rt2);
        }
    }
  //Draw points
  if (drawPointList){
      QMap<unsigned int, coloredPoint>::iterator lp;
      for(lp=this->pointList.begin(); lp != this->pointList.end(); ++lp){
          QPen myPen(lp.value().color);
          myPen.setWidth(lp.value().width);
          painter.setPen(myPen);
          painter.drawPoint(QPoint(xToPix(lp.value().point.x()),yToPix(lp.value().point.y())));
        }
      for(lp=this->secondPointList.begin(); lp != this->secondPointList.end(); ++lp){
          QPen myPen(lp.value().color);
          myPen.setWidth(lp.value().width);
          painter.setPen(myPen);
          painter.drawPoint(QPoint(xToPix(lp.value().point.x()),yToPix(lp.value().point.y())));
        }
      painter.setPen(lPen);
    }
  painter.setPen(lPen);
  //Draw bold points
  for(lp=this->boldPointList.begin(); lp != this->boldPointList.end(); ++lp){
      QPen myBoldPen(lp->color);
      myBoldPen.setWidth(3);
      painter.setPen(myBoldPen);
      painter.drawPoint(QPoint(xToPix(lp->point.x()),yToPix(lp->point.y())));
    }
  painter.setPen(lPen);
  //Draw Tubes
  vector<Interval>::const_iterator it;
  for (int i = 0 ; i < tubeList.size() ; i++){
      QColor myColor = tubeList.at(i).color;
      if (tubeList.size() > 1)
        myColor.setAlpha(100);
      double t=tubeList.at(i).startTime;
      double dt = tubeList.at(i).tube.dt;
      QRect r;
      for(it=this->tubeList.at(i).tube.begin(); it != this->tubeList.at(i).tube.end(); it++)
        {
          r.setTopLeft(QPoint(xToPix(t),yToPix(it->sup)));
          r.setBottomRight(QPoint(xToPix(t+dt),yToPix(it->inf)));
          painter.fillRect(r,myColor);
          painter.drawRect(r);
          t += dt;
        }
    }
  painter.setPen(lPen);
  //Draw Newton tested boxes
  if (!newtonTestedBox.box.isEmpty()){
      painter.setPen(QPen(newtonTestedBox.color));
      QRect r;
      r.setTopLeft(QPoint(xToPix(newtonTestedBox.box[1].inf),yToPix(newtonTestedBox.box[2].sup)));
      r.setBottomRight(QPoint(xToPix(newtonTestedBox.box[1].sup),yToPix(newtonTestedBox.box[2].inf)));
      painter.drawRect(r);
      painter.setPen(lPen);
    }
  //Draw Newton contracted boxes
  if (!newtonContractedBox.box.isEmpty()){
      painter.setPen(QPen(newtonContractedBox.color));
      QRect r;
      r.setTopLeft(QPoint(xToPix(newtonContractedBox.box[1].inf),yToPix(newtonContractedBox.box[2].sup)));
      r.setBottomRight(QPoint(xToPix(newtonContractedBox.box[1].sup),yToPix(newtonContractedBox.box[2].inf)));
      painter.drawRect(r);
      painter.setPen(lPen);
    }
  //Draw Newton boxes
  QList<coloredRect>::iterator lin;
  for(lin=this->newtonList.begin(); lin != this->newtonList.end(); ++lin)
    {
      QPen myNewPen(QPen(lin->color));
      myNewPen.setWidth(2);
      painter.setPen(myNewPen);
      QRect r;
      r.setTopLeft(QPoint(xToPix(lin->box[1].inf),yToPix(lin->box[2].sup)));
      r.setBottomRight(QPoint(xToPix(lin->box[1].sup),yToPix(lin->box[2].inf)));
      painter.drawRect(r);
    }
  painter.setPen(lPen);
  //draw Zero horizontal Line
  if (drawZeroLine){
      painter.setPen(Qt::darkGray);
      painter.setPen(Qt::DotLine);
      painter.drawLine(QPoint(xToPix(0),yToPix(0)),QPoint(xToPix(priorDomain[1].sup),yToPix(0)));
      painter.setPen(lPen);
    }
  painter.setPen(lPen);
  //drawDiagonal Line
  if (drawDiagonal && !initDom.isEmpty()){
      painter.setPen(Qt::darkGray);
      painter.setPen(Qt::DotLine);
      painter.drawLine(QPoint(xToPix(0),yToPix(0)),QPoint(xToPix(initDom[1].sup),yToPix(initDom[2].sup)));
      painter.setPen(lPen);
    }
//  //Try to complete between Kalman points
//  /**
//    * 1) d'un point à un autre, compter le nombre de pixels
//    * 2) from http://stackoverflow.com/questions/20655174/qt-how-do-i-make-a-field-of-2d-interpolated-colors
//    * c00   -- x -->  c01

//        |
//        |
//        y      c(x,y)
//        |
//        |
//        V

//      c10              c11
//    *
//    * c0=c(x,0)=c00+((c01-c00)*x)
//    * c1=c(x,1)=c10+((c11-c10)*x)
//    * c(x,y)   =c0 +((c1 -c0 )*y)
//    *
//    *
//   */
//  if (drawKalmanResults)
//    {
//      for(lp=this->kalmanPointList.begin(); lp != this->kalmanPointList.end(); ++lp){
////          QPen myBoldPen(lp->color);
////          myBoldPen.setWidth(lp->width);
////          painter.setPen(myBoldPen);
////          painter.drawPoint(QPoint(xToPix(lp->point.x()),yToPix(lp->point.y())));
//        }
//      painter.setPen(lPen);
//    }
}

void Frame::wheelEvent(QWheelEvent * event)
{

  //le scroll zoom/dezoom sur le Frame.
  //1 - On récupère le centre de la souris
  QPoint pos = event->globalPos();
  zoom(pos,event->delta());
}

void Frame::zoom(QPoint pos, int alpha)
{
  pos = mapFromGlobal(pos);
  //2 - On le transforme dans notre repère
  double x = pixToX(pos.x());
  double y = pixToY(pos.y());
  //Si on est en Zoom ou en deZoom
  if (alpha < 0){
      //On est en dezoom
      Box b = getPriorDomain();
      float c = 1.5;
      Interval x1 = Interval(x-c*b[1].width()/2,x+c*b[1].width()/2);
      Interval y1 = Interval(y-c*b[2].width()/2,y+c*b[2].width()/2);
      //Le nouveau domaine est l'ancien "dégonflé" de 10%
      //L'intersection sert a respecter les domaines initiaux
      Box ttt = initDom;
      b[1] = inter(x1,initDom[1]);
      b[2] = inter(y1,initDom[2]);
      float maxWidth  = max(b[1].width(),b[2].width());
      b[1].inf = b[1].center() - maxWidth/2;
      b[1].sup = b[1].center() + maxWidth/2;
      b[2].inf = b[2].center() - maxWidth/2;
      b[2].sup = b[2].center() + maxWidth/2;
      if (!(isnan(b[1].inf) || isnan(b[1].sup) || isnan(b[2].inf) ||isnan(b[2].sup)))
        setPriorDomain(b);
    }else if (alpha > 0){
      //On est en zoom
      Box b = getPriorDomain();
      float c = 0.5;
      Interval x1 = Interval(x-c*b[1].width()/2,x+c*b[1].width()/2);
      Interval y1 = Interval(y-c*b[2].width()/2,y+c*b[2].width()/2);
      //Le nouveau domaine est l'ancien "dégonflé" de 10%
      //L'intersection sert a respecter les domaines initiaux
      b[1] = inter(x1,initDom[1]);
      b[2] = inter(y1,initDom[2]);
      float maxWidth  = max(b[1].width(),b[2].width());
      b[1].inf = b[1].center() - maxWidth/2;
      b[1].sup = b[1].center() + maxWidth/2;
      b[2].inf = b[2].center() - maxWidth/2;
      b[2].sup = b[2].center() + maxWidth/2;
      if (!(isnan(b[1].inf) || isnan(b[1].sup) || isnan(b[2].inf) ||isnan(b[2].sup)))
        setPriorDomain(b);
    }
}
void Frame::updateAxisToData(void)
{
  // get min and max from tubeList;
  QList<coloredTube>::iterator it1;
  vector<double> minOrd;
  vector<double> maxOrd;
  vector<double> minAbs;
  vector<double> maxAbs;
  for (it1 = tubeList.begin() ; it1 != tubeList.end() ; it1++){
      minAbs.push_back(0);
      minOrd.push_back(minNotoo(it1->tube));
      maxOrd.push_back(maxNotoo(it1->tube));
      maxAbs.push_back(it1->tube.size()*it1->tube.dt);
    }
  // get min and max from boxList;
  QMap<unsigned int,coloredRect>::iterator it2;
  double minAbsB = +oo;
  double maxAbsB = -oo;
  double minOrdB = +oo;
  double maxOrdB = -oo;
  for (it2 = boxList.begin() ; it2 != boxList.end() ; it2++){
      if (it2.value().box[0].inf < minAbsB)
        minAbsB = it2.value().box[0].inf;
      if (it2.value().box[0].sup > maxAbsB)
        maxAbsB = it2.value().box[0].sup;
      if (it2.value().box[1].inf < minOrdB)
        minOrdB = it2.value().box[1].inf;
      if (it2.value().box[1].sup > maxOrdB)
        maxOrdB = it2.value().box[1].sup;
    }
  if (minOrdB != +oo)
    minOrd.push_back(minOrdB);
  if (maxOrdB != -oo)
    maxOrd.push_back(maxOrdB);
  if (minAbsB != +oo)
    minAbs.push_back(minAbsB);
  if (maxAbsB != -oo)
    maxAbs.push_back(maxAbsB);
  // get min and max from pointList;
  QMap<unsigned int, coloredPoint>::iterator it3;
  minAbsB = +oo;
  maxAbsB = -oo;
  minOrdB = +oo;
  maxOrdB = -oo;
  for (it3 = pointList.begin() ; it3 != pointList.end() ; it3++){
      if (it3.value().point.x() < minAbsB)
        minAbsB = it3.value().point.x();
      if (it3.value().point.x() > maxAbsB)
        maxAbsB = it3.value().point.x();
      if (it3.value().point.y() < minOrdB)
        minOrdB = it3.value().point.y();
      if (it3.value().point.y() > maxOrdB)
        maxOrdB = it3.value().point.y();
    }
  if (minOrdB != +oo)
    minOrd.push_back(minOrdB);
  if (maxOrdB != -oo)
    maxOrd.push_back(maxOrdB);
  if (minAbsB != +oo)
    minAbs.push_back(minAbsB);
  if (maxAbsB != -oo)
    maxAbs.push_back(maxAbsB);

  vector<double>::iterator itMinAbs = std::min_element(minAbs.begin(),minAbs.end(),std::less<double>());;
  vector<double>::iterator itMaxAbs = std::max_element(maxAbs.begin(),maxAbs.end(),std::greater<double>());;
  vector<double>::iterator itMinOrd = std::min_element(minOrd.begin(),minOrd.end(),std::less<double>());;
  vector<double>::iterator itMaxOrd = std::max_element(maxOrd.begin(),maxOrd.end(),std::greater<double>());;

  Box b(Interval(*itMinAbs, *itMaxAbs),Interval(*itMinOrd, *itMaxOrd));

  //    Interval SPD = b[1];
  //    Interval SPD2 = b[2];
  //  qDebug()<<"setPriorDomain";
  //  qDebug()<<'['<<SPD.inf<<'|'<<SPD.center()<<'|'<<SPD.sup<<"], min is inf?"<<isinf(SPD.inf)<<isnan(SPD.inf);
  //  qDebug()<<'['<<SPD2.inf<<'|'<<SPD2.center()<<'|'<<SPD2.sup<<"], is empty?"<<SPD2.isEmpty();

  setInitialDomain(b);
}
void Frame::saveImage(std::string fileName)
{
  //  qDebug()<<myFileName;
  QString myText(QString("Do you want to save the image as [")+QString(fileName.c_str())+QString("] ?"));
  //We have to save the file
  QMessageBox msgBox;
  msgBox.setText(myText);
  msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::Cancel);
  msgBox.setDefaultButton(QMessageBox::Save);
  int ret = msgBox.exec();
  QSize imgSize(this->size());
  QImage img(imgSize,QImage::Format_ARGB32_Premultiplied);
  img.setDotsPerMeterX(600);
  img.setDotsPerMeterY(600);
  QPainter p(&img);
  bool t;
  switch (ret) {
    case QMessageBox::Save:
      render(&p);
      p.end();
      img = img.scaled(1024,1024,Qt::KeepAspectRatio);
      img.setDotsPerMeterX(600);
      img.setDotsPerMeterY(600);
      t = img.save(fileName.c_str());
      if (!t){
          msgBox.setText("Problème a la sauvegarde de l'image");
          msgBox.setStandardButtons(QMessageBox::Ok);
          msgBox.setDefaultButton(QMessageBox::Ok);
          ret = msgBox.exec();
        }
      //      qDebug()<<"probleme dans le save...";
      break;
    case QMessageBox::Cancel:
      break;
    default:
      // should never be reached
      break;
    }
}
