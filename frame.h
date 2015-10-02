#ifndef FRAME_H
#define FRAME_H

#include <QWidget>
#include <QList>
#include <QPainter>
#include <QLabel>
#include <QWheelEvent>
#include "interval/box.h"
#include "interval/scalarTube.h"

struct coloredRect {
  Box box;
  QColor color;
};
struct coloredPoint {
  QPointF point;
  QColor color;
  int width;
};
struct coloredTube {
  ScalarTube tube;
  int startTime;
  QColor color;
};


class Frame : public QWidget
{
  Q_OBJECT
public:
  Frame(QLabel * abscMinLbl, QLabel * abscMeanLbl, QLabel * abscMaxLbl, QLabel * ordMinLbl, QLabel * ordMeanLbl, QLabel * ordMaxLbl, Box & b, bool pDrawRect, QWidget *parent = 0, QPen pPen = Qt::NoPen);
  void resizeEvent(QResizeEvent * event);
  void setPriorDomain(Box&);
  void setInitialDomain(Box&);
  double xToPix(double );
  double yToPix(double );
  double pixToX(double );
  double pixToY(double );
  inline Box getPriorDomain(void) const {
    return priorDomain;
  }
  inline Box getInitialDomain(void) const {
    return initDom;
  }
  inline bool getDrawRect(void) const {
    return drawRect;
  }
  inline void setDrawRect(const bool pDrawRect) {
    drawRect = pDrawRect;
    update();
  }
  inline void setDrawClickedBox(const bool pDrawClickedBox) {
    drawClickedBox = pDrawClickedBox;
    update();
  }
  inline void setPen(QPen p) {
    lPen = p;
  }
  inline void setPlotGrayScale(bool p) {
    plotGrayScale = p;
  }
  inline void setDrawZeroLine(bool p) {
    drawZeroLine = p;
  }
  inline void setDrawDiagonal(bool p) {
    drawDiagonal = p;
  }
  void setBoxAtT1(Box&,QColor);
  void setBoxAtT2(Box&,QColor);
  inline void setMyGeometry(int w, int h) {
    this->setGeometry(0,0,w, h);
  }
  void saveImage(std::string graphName);
  void make_grayscale(QImage& in);

public slots:
  void addTube(ScalarTube&,int startTime,QColor);
  void addBox(unsigned int, Box&, QColor);
  void addNewtonBox(Box &b,QColor);
  void plotNewtonTestedBox(Box &b,QColor);
  void plotNewtonContractedBox(Box &b,QColor);
  void addBoldPoint(QPointF&,QColor);
  void addBoldPoint(double, double, QColor);
  void addKalmanPoint(double, double, QColor);
  void addPoint(unsigned int, QPointF&,QColor);
  void addPoint(unsigned int, double, double, QColor);
  void addSecondPoint(unsigned int, double, double, QColor);
  void clear();
  void updateAxisToData(void);
  void zoom(QPoint pos, int alpha);


protected:
  void paintEvent(QPaintEvent * event);
  void wheelEvent(QWheelEvent * event);

public:
  //Things to print
  coloredRect  newtonTestedBox;
  coloredRect  newtonContractedBox;
  QList<coloredTube>  tubeList;
  QMap<unsigned int,coloredRect>  boxList;
  QMap<unsigned int,coloredPoint> pointList;
  QMap<unsigned int,coloredPoint> secondPointList;
  QList<coloredPoint> boldPointList;
  QList<coloredPoint>  kalmanPointList;
  QList<coloredRect>  newtonList;
  //Prior domain
  Box priorDomain;
  //scale factors
  double echx;
  double echy;
  //Selected box/times
  coloredRect boxAtT1;
  coloredRect boxAtT2;
  bool drawClickedBox;

  //Pointer to labels of axes and title
  QLabel * pTitleLbl;
  QLabel * pAbsMinLbl;
  QLabel * pAbsMeanLbl;
  QLabel * pAbsMaxLbl;
  QLabel * pOrdMinLbl;
  QLabel * pOrdMeanLbl;
  QLabel * pOrdMaxLbl;

  //Draw lines of the box?
  QPen lPen;
  bool drawRect;
  //Draw horizontal zero line?
  bool drawZeroLine;
  bool drawDiagonal;
  bool drawPointList;
  bool plotGrayScale;
  bool drawKalmanResults;
  bool drawLDResults;

  //Initial domain for zoom function
  Box initDom;
};




#endif // FRAME_H
