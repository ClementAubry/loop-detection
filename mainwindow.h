#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QMouseEvent>
#include <QTimer>

#include "interval/box.h"
#include "frame.h"
#include "loopdetector.h"

static const std::string pathToData = "./";

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    //Constructor
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
    /**
      * cheatFramesResolution function is a dirty cheat for frames to fit in QGraphicsView
      */
    void cheatFramesResolution(void);

protected:
    /**
      * changeEvent function is a built in function for language translation
      * @param e an event
      */
    void changeEvent(QEvent *e);
    /**
      * drawDatas function draw velocity/position tubes and position boxes
      */
    void drawDatas(void);
    /**
      * findLoop function perform the loop detection algorithm
      * @param precSivia the precision of the SIVIA like algorithm
      */
    void findLoop(double precSivia);
    /**
      * mousePressEvent function intercept a click on tPlane and show clicked times in other frames
      * @param event a QMouseEvent
      */
    void mouseMoveEvent(QMouseEvent * event);
    void mousePressEvent(QMouseEvent * event);
    void mouseReleaseEvent(QMouseEvent * event);
    void keyPressEvent(QKeyEvent *event);
    /**
      * setTrajFromFile function read a data file and put it in tubes
      * @param pTrajFile a string containing the name of the file
      * @param nbLines the number of lines you want to read in the file, default value = -1 to read all lines
      * @return true if the file has been successfully read, false if not
      */
    bool setTrajFromFile(const std::string & pTrajFile, int nbLines = -1);

    bool getKalmanResults(const std::string & pTrajFile);
    /**
      * resizeEvent function is an overloaded member and just call cheatFrameResolution
      * @param event an event
      */
    void resizeEvent(QResizeEvent * event);

private slots:
    //Check if we had to draw boxes borders
    void drawBoxesBorder(void);
    void drawBoxesCenter(void);
    void saveGraphs(void);

    //Start loop detection
    void startLoopDetection(void);
    //Draw datas clicked from Matlab OR Redermor mode
    void drawDataClicked();
    //compute and draw LDKalman by euclidean distance
    void computeKalmanTPlaneDist();
    void computeKalmanTPlaneDistSeuilSeeDelay();

    //reset axis to priors domains of each graphs
    void resetAxes();
    //clear axes and tubes if computed
    void clearAxes(void);
    //setColorMode
    void setColorMode(void);

    void on_drawDiagonalcheckBox_clicked();

    void on_zoomInToolButton_clicked();

    void on_zoomOutToolButton_clicked();

    void on_zoomResetToolButton_clicked();

    void on_newtonCheckBox_clicked();

    void on_newtonToolButton_clicked();

    void on_kalmanCheckBox_clicked();

    void on_seuilDistKalmanStepSpinBox_editingFinished();

    void on_seuilDistKalmanStepSpinBox_valueChanged(int arg1);

    void on_plotEnsemblisteCheckBox_clicked();

    void on_toposDegToolButton_clicked();

private:
    //Main window
    Ui::MainWindow *ui;
    //Graphic frames
    Frame * plotTime;
    Frame * plotPosition;
    Frame * plotIntegral;
    //les 'Global' sont les vecteurs qui contiennent tous les points, les autres sont le sous echantillonage
    vector<double> estimXGlobal;
    vector<double> estimX;
    vector<double> estimYGlobal;
    vector<double> estimY;

    //Tubes
    //les 'Global' sont les vecteurs qui contiennent tous les points, les autres sont le sous echantillonage
    ScalarTube * pTubeDxGlobal;
    ScalarTube * pTubeDyGlobal;
    ScalarTube * pTubeX;
    ScalarTube * pTubeY;
    ScalarTube * pTubeDx;
    ScalarTube * pTubeDy;
    //true when tubes correctly setted up
    bool tubesComputed;
    bool pDrawBoxesCenter;
    //For SIVIA
    LoopDetector * myLoopDetector;
    //We keep two points in the t-plane to manually test unicity by Newton operator
    QPoint firstPoint;
    QPoint secondPoint;
    bool selectingBox;
    bool movingPlane;
//    int stepIntegralTube;

    //ColorMode
    QColor outside;
    QColor inside;
    QColor positions;
    QColor NTested;
    QColor NResult;
    QColor NPassed;

    //For tool buttons
    bool zoomInActive;
    bool zoomOutActive;
    bool newtonToolBtActive;
    bool topoDegBtActive;

};

#endif // MAINWINDOW_H
