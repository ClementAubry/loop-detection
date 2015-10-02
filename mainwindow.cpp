#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <stdio.h>
#include <algorithm>    //for min_element
#include <functional>   // for less
#include <QApplication>
#include <QDesktopWidget>
#include <fstream>
#include <QMessageBox>
#include <QDebug>
#include <QTime>
#include <QScrollBar>
#include "utils.h"
#ifdef WIN32
#include <ctime>
#endif

using std::min;
using std::max;
using std::vector;
using std::list;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);//Main window configuration
    setCursor(Qt::CrossCursor);
    setFocusPolicy(Qt::ClickFocus);//Focus activated on click
    setlocale(LC_NUMERIC,"C");//Float reading convention ('.' instead of ',' for floating point numbers)
    //Graphics initialisation
    Box initBox = Box(Interval(0,10),Interval(0,10));
    plotTime        = new Frame(ui->t1MinLabel, ui->t1MedLabel, ui->t1MaxLabel, ui->t2MinLabel, ui->t2MedLabel, ui->t2MaxLabel, initBox, ui->drawRectChk->isChecked(), ui->timeGraph, QPen(Qt::darkGray));
    plotTime->setDrawDiagonal(ui->drawDiagonalcheckBox->isChecked());
    plotTime->drawLDResults = ui->plotEnsemblisteCheckBox->isChecked();
    plotTime->drawKalmanResults = ui->kalmanCheckBox->isChecked();
    plotPosition    = new Frame(ui->xMinLabel_1, ui->xMedLabel_1, ui->xMaxLabel_1, ui->yMinLabel_1, ui->yMedLabel_1, ui->yMaxLabel_1, initBox, ui->drawRectChk->isChecked(), ui->positionGraph);
    plotPosition->setDrawDiagonal(false);
    plotPosition->drawPointList = ui->drawBoxCenterChkBox->isChecked();
    plotPosition->drawLDResults = true;
    plotIntegral = new Frame(ui->xMinIntegralLbl, ui->xMedIntegralLbl, ui->xMaxIntegralLbl, ui->yMinIntegralLbl, ui->yMedIntegralLbl, ui->yMaxIntegralLbl, initBox, ui->drawRectChk->isChecked(), ui->IntegralGraph);
    plotIntegral->setDrawZeroLine(true);
    plotIntegral->setDrawDiagonal(false);
    //SLOT connections
    connect(ui->resetButton, SIGNAL(clicked()), this, SLOT(resetAxes()));
    connect(ui->clearButton, SIGNAL(clicked()), this, SLOT(clearAxes()));
    connect(ui->loopBt, SIGNAL(clicked()), this, SLOT(startLoopDetection()));
    connect(ui->drawBt, SIGNAL(clicked()), this, SLOT(drawDataClicked()));
    connect(ui->drawRectChk, SIGNAL(clicked()), this, SLOT(drawBoxesBorder()));
    connect(ui->drawBoxCenterChkBox, SIGNAL(clicked()), this, SLOT(drawBoxesCenter()));
    connect(ui->saveButton, SIGNAL(clicked()), this, SLOT(saveGraphs()));
    connect(ui->grayScaleChkBox, SIGNAL(clicked()), this, SLOT(setColorMode()));
    //At start, nothing is done
    tubesComputed = false;
    selectingBox = false;
    movingPlane = false;
    pDrawBoxesCenter = ui->drawBoxCenterChkBox->isChecked();
    //pointers survey..
    myLoopDetector = NULL;
    pTubeX = NULL;
    pTubeY = NULL;
    pTubeDx = NULL;
    pTubeDy = NULL;
    pTubeDxGlobal = NULL;
    pTubeDyGlobal = NULL;
    //a dirty cheat for resolution and display
    cheatFramesResolution();
    //Disable unusable buttons
    ui->saveButton->setEnabled(false);
    ui->loopBt->setEnabled(false);
    ui->grayScaleChkBox->setEnabled(true);
    setColorMode();
    //Load datas
    time_t t1,t2;
    time(&t1);
    //Read data file
    QString myFile = QString(pathToData.c_str())+QString("data_nav.txt");
    tubesComputed = setTrajFromFile(myFile.toStdString().c_str(),60000);
    if (!tubesComputed){
        QMessageBox::information(this, "Erreur", "Mauvaise lecture du fichier");
        return;
    }
    QString myFileK = QString(pathToData.c_str())+QString("svgXHat.txt");
    bool kalmanComputed = getKalmanResults(myFileK.toStdString().c_str());
    if (!kalmanComputed){
        QMessageBox::information(this, "Erreur", "Mauvaise lecture du fichier Kalman");
        return;
    }
    tubesComputed = false;
    time(&t2);
    ui->infoBox->insertPlainText(QString::number(pTubeDxGlobal->size(),'d',0)+" Redermor datas loaded in "+QString::number(difftime(t2,t1),'d',0)+" seconds");
    zoomInActive = ui->zoomInToolButton->isDown();
    zoomOutActive = ui->zoomOutToolButton->isDown();
    newtonToolBtActive = ui->newtonToolButton->isDown();
    drawDataClicked();
    //    //NEW C.A. get kalman results
    //    computeKalmanTPlaneDist(); //could be founded in startLoopDetection function
    startLoopDetection();
    QScrollBar *sb = ui->infoBox->verticalScrollBar();
    sb->setValue(sb->maximum());

}
MainWindow::~MainWindow()
{
    delete plotTime;
    delete plotPosition;
    delete plotIntegral;
    delete ui;
}
//PROTECTED MEMBERS
/**
  * changeEvent function is a built in function for language translation
  * @param e an event
  */
void MainWindow::changeEvent(QEvent *e)
{
    QMainWindow::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}
/**
  * drawDatas function draw velocity/position tubes and position boxes
  */
void MainWindow::drawDatas(void)
{
    //Just check if we had to draw boxes borders DIRTY
    drawBoxesBorder();
    //setup position axes
    Interval x(minNotoo(*pTubeX), maxNotoo(*pTubeX));
    Interval y(minNotoo(*pTubeY), maxNotoo(*pTubeY));
    //    Box b(y,x);
    Box b(x,y);
    plotPosition->setInitialDomain(b);
    for(unsigned int i = 0 ; i < pTubeX->size() ; i++){
        //plot Position => centers of each boxes
        //        plotPosition->addPoint(i,pTubeY->at(i).center(),pTubeX->at(i).center(),Qt::black);
        plotPosition->addPoint(i,pTubeX->at(i).center(),pTubeY->at(i).center(),Qt::black);
        //plot Position => each boxes
        //        Box myBox = Box(pTubeY->at(i),pTubeX->at(i));
        Box myBox = Box(pTubeX->at(i),pTubeY->at(i));
        plotPosition->addBox(i, myBox,positions);
    }
    //Data are known here, so we can initialize our t-plane
    Box myBox(Interval(0,pTubeDx->size()-1),Interval(0,pTubeDx->size()-1));
    plotTime->setInitialDomain(myBox);

    // Uggly output to be compared with some matlab version of the algorithm
    //    ui->infoBox->insertPlainText("\ntubeDX.size()=["+QString::number(pTubeDx->size(),'f',5));
    //    ui->infoBox->insertPlainText("\ntubeDX.dt=["+QString::number(pTubeDx->dt,'f',5));
    //    ui->infoBox->insertPlainText("\ntubeDX(0)=["+QString::number(pTubeDx->at(0).inf,'f',5)+";"+QString::number(pTubeDx->at(0).sup,'f',5)+"]");
    //    ui->infoBox->insertPlainText("\ntubeDX(1)=["+QString::number(pTubeDx->at(1).inf,'f',5)+";"+QString::number(pTubeDx->at(1).sup,'f',5)+"]");
    //    ui->infoBox->insertPlainText("\ntubeDX(2)=["+QString::number(pTubeDx->at(2).inf,'f',5)+";"+QString::number(pTubeDx->at(2).sup,'f',5)+"]");
    //    ui->infoBox->insertPlainText("\ntubeDX(end)=["+QString::number(pTubeDx->at(pTubeDx->size()-1).inf,'f',5)+";"+QString::number(pTubeDx->at(pTubeDx->size()-1).sup,'f',5)+"]");
    //    ui->infoBox->insertPlainText("\n--------------");
    //    ui->infoBox->insertPlainText("\ntubeX(0)=["+QString::number(pTubeX->at(0).inf,'f',5)+";"+QString::number(pTubeX->at(0).sup,'f',5)+"]");
    //    ui->infoBox->insertPlainText("\ntubeX(1)=["+QString::number(pTubeX->at(1).inf,'f',5)+";"+QString::number(pTubeX->at(1).sup,'f',5)+"]");
    //    ui->infoBox->insertPlainText("\ntubeX(2)=["+QString::number(pTubeX->at(2).inf,'f',5)+";"+QString::number(pTubeX->at(2).sup,'f',5)+"]");
    //    ui->infoBox->insertPlainText("\ntubeX(end)=["+QString::number(pTubeX->at(pTubeX->size()-1).inf,'f',5)+";"+QString::number(pTubeX->at(pTubeX->size()-1).sup,'f',5)+"]");

}
/**
  * findLoop function perform the loop detection algorithm
  * @param precSivia the precision of the SIVIA like algorithm
  */
void MainWindow::findLoop(double precSivia)
{
    //"profiling" purpose
    QTime temps;
    temps.start();
    //
    //display infos
//    ui->infoBox->insertPlainText(QString("\nLoop detection starts at : "+ QTime::currentTime().toString()));
    // Loop detection algorithm (SIVIA like) only needs velocity tubes
    if (myLoopDetector != NULL){
        delete myLoopDetector;
        myLoopDetector = NULL;
    }
    myLoopDetector = new LoopDetector(precSivia, pTubeDx, pTubeDy, pTubeX, pTubeY);
    //Launch Loop Detection Algorithm
    myLoopDetector->resolve();
//    ui->infoBox->insertPlainText(QString("\nLoop detection end at : "+ QTime::currentTime().toString()));
    ui->infoBox->insertPlainText("\nLoop detection resolved in "+QString::number(temps.elapsed())+" ms");
    //Get results
    list<Box> NO =  myLoopDetector->getNotSolutions();
    list<Box> YES =  myLoopDetector->getSolutions();
    list<Box> PE =  myLoopDetector->getPerhaps();
    list<Box> newton =  myLoopDetector->getNewton();
    list<Box>::iterator it;
    //And print them out
    temps.restart();
    setColorMode();
    unsigned int idBoxForMap = 0;
    double volumeNO = 0, volumePE = 0, volumeYES = 0;
    for (it = NO.begin(); it != NO.end() ; it++){
        plotTime->addBox(idBoxForMap++, *it,outside);
        volumeNO += (*it).volume();
    }
    for (it = YES.begin(); it != YES.end() ; it++){
        plotTime->addBox(idBoxForMap++, *it,inside);
        volumeYES += (*it).volume();
    }
    for (it = PE.begin(); it != PE.end() ; it++){
        plotTime->addBox(idBoxForMap++, *it,Qt::yellow);
        volumePE += (*it).volume();
    }
    for (it = newton.begin(); it != newton.end() ; it++){
        plotTime->addNewtonBox(*it,NPassed);
    }
    plotTime->update();
    //display infos
    ui->infoBox->insertPlainText("\nLoop detection results drawn in "+QString::number(temps.elapsed())+" ms");
    ui->infoBox->insertPlainText("\n----------------------------------------------------------");
    //Print percentage of the t-plane used by each sivia result domains (tin, T?, Tout)
    double volumeFull = plotTime->getPriorDomain().volume();
    ui->infoBox->insertPlainText("\nTin : " + QString::number(volumeYES/volumeFull*100,'f',5)+"% of the t-plane, " + QString::number(YES.size(),'d',0)+" boxes");
    ui->infoBox->insertPlainText("\nT? : "  + QString::number(volumePE/volumeFull*100,'f',5) +"% of the t-plane " + QString::number(PE.size(),'d',0)+" boxes");
    ui->infoBox->insertPlainText("\nTout : "+ QString::number(volumeNO/volumeFull*100,'f',5) +"% of the t-plane " + QString::number(NO.size(),'d',0)+" boxes");
    ui->infoBox->insertPlainText("\n----------------------------------------------------- -----");
    //Newton is not yet automatic, we manually check known Newton boxes
    vector<Box > manualNewtonBoxes;
    //Pour LUC JAULIN : la liste suivante est celle de toutes les boites que j'ai réussi a valider par Newton a la main. A décommenter pour qu'elles soient caculées automatiquement
    /*
    manualNewtonBoxes.push_back(Box(Interval(29441,31950),Interval(55308,57862)));//ta
    manualNewtonBoxes.push_back(Box(Interval(10420,11321),Interval(29228,30407)));//tb
    manualNewtonBoxes.push_back(Box(Interval(17124,17876),Interval(25362,26355)));//tc
    manualNewtonBoxes.push_back(Box(Interval(13219,14224),Interval(30196,31340)));
    manualNewtonBoxes.push_back(Box(Interval(20996,21759),Interval(30542,31685)));
    manualNewtonBoxes.push_back(Box(Interval(48617,49073),Interval(50896,51324)));
    manualNewtonBoxes.push_back(Box(Interval(10051,11283),Interval(25916,27468)));
    manualNewtonBoxes.push_back(Box(Interval(17921,18671),Interval(29946,30779)));
    manualNewtonBoxes.push_back(Box(Interval(26963,27139),Interval(29674,29818)));
    manualNewtonBoxes.push_back(Box(Interval(43516,44092),Interval(51928,52551)));
    manualNewtonBoxes.push_back(Box(Interval(40448,41171),Interval(51377,52177)));
    manualNewtonBoxes.push_back(Box(Interval(36304,37270),Interval(52297,53405)));
    manualNewtonBoxes.push_back(Box(Interval(25739,27050),Interval(56609,57692)));
    manualNewtonBoxes.push_back(Box(Interval(20298,21640),Interval(55211,56896)));
    */
    vector<Box >::iterator itN;
    Box domain = plotTime->getPriorDomain();
    for (itN = manualNewtonBoxes.begin() ; itN < manualNewtonBoxes.end() ; itN++){
        //Eeach box in the domain
        if ((*itN).isIn(domain) == itrue){
            //is tested by Newton
            Box N = myLoopDetector->newtonTest(*itN, *ui->infoBox);
            if ( !N.isEmpty() && N.isIn(*itN) == itrue ){
                //draw newton box
                plotTime->addBox(plotTime->boxList.count()+1,(*itN),Qt::blue);
                plotTime->addNewtonBox(N, NPassed);
                //print out
                ui->infoBox->insertPlainText("\nNewton(["+QString::number((*itN)[1].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+(*itN)[1].sup*pTubeDx->dt,'f',1)+"]["+QString::number((*itN)[2].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+(*itN)[2].sup*pTubeDx->dt,'f',1)+"])");
                ui->infoBox->insertPlainText("= ["+QString::number(N[1].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+N[1].sup*pTubeDx->dt,'f',1)+"]["+QString::number(N[2].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+N[2].sup*pTubeDx->dt,'f',1)+"]");
            }
        }
    }
    /**/
    plotTime->update();
    //And enable some features
    ui->saveButton->setEnabled(true);
    ui->loopBt->setEnabled(false);
    //Refresh graphics
    plotPosition->update();

    //Plot Integral tubes of the entire mission
    plotIntegral->addTube(*pTubeX,0,Qt::blue);
    plotIntegral->addTube(*pTubeY,0,Qt::green);
    //Compute best axis for this plot
    Interval i1 = pTubeX->intervalEvaluation(0,pTubeX->size()-1);
    Interval i2 = pTubeY->intervalEvaluation(0,pTubeY->size()-1);
    Interval i3 = hull(i1,i2);
    Box myBox(Interval(0,(pTubeY->size()-1)*pTubeY->dt),i3);
    plotIntegral->setInitialDomain(myBox);
    QScrollBar *sb = ui->infoBox->verticalScrollBar();
    sb->setValue(sb->maximum());
}
void MainWindow::mouseMoveEvent(QMouseEvent * event)
{
    QPoint myPoint = ui->timeGraph->mapFromGlobal(event->globalPos());
    secondPoint.setX(plotTime->pixToX(myPoint.x()));
    secondPoint.setY(plotTime->pixToY(myPoint.y()));
    if (selectingBox && ui->timeGraph->hasFocus() && tubesComputed){
        //build the selected box
        Interval t1(floor(min(firstPoint.rx(),secondPoint.rx())),ceil(max(firstPoint.rx(),secondPoint.rx())));
        Interval t2(floor(min(firstPoint.ry(),secondPoint.ry())),ceil(max(firstPoint.ry(),secondPoint.ry())));
        Box T (t1,t2);
        if (event->modifiers() == Qt::ControlModifier || zoomInActive || zoomOutActive){
            plotTime->plotNewtonTestedBox(T,Qt::black);
        }else if(event->modifiers() == Qt::ShiftModifier || newtonToolBtActive){
            plotTime->plotNewtonTestedBox(T,NTested);
        }else{
            Box T;
            plotTime->plotNewtonTestedBox(T,NTested);
        }
        plotTime->update();
    }else if(movingPlane && ui->timeGraph->hasFocus() && tubesComputed){
        //On veut faire glisser le tplane...
        //TODO!!!
    }else if (ui->timeGraph->hasFocus() && tubesComputed){
        //On n'est plus en train de selectionner une box, on veut afficher le tube position
        if (!plotIntegral->tubeList.empty()){
            plotIntegral->tubeList.clear();
        }
        //        unsigned int t1c = secondPoint.x();
        //        unsigned int t2c = secondPoint.y();
        //Pour pouvoir se deplacer dans le t-plane mm si t1>t2
        unsigned int t1c = min(secondPoint.x(),secondPoint.y());
        unsigned int t2c = max(secondPoint.x(),secondPoint.y());
        unsigned int step = 1 + (t2c - t1c)/100;
        ScalarTube integral_t1_t_vx = pTubeDx->timeIntegrationTubeLessData(t1c,t2c,step);
        ScalarTube integral_t1_t_vy = pTubeDy->timeIntegrationTubeLessData(t1c,t2c,step);
        plotIntegral->addTube(integral_t1_t_vx,pTubeDx->dt*t1c,Qt::blue);
        plotIntegral->addTube(integral_t1_t_vy,pTubeDy->dt*t1c,Qt::green);
        //Compute best axis for this plot
        Interval i1 = integral_t1_t_vx.intervalEvaluation(0,integral_t1_t_vx.size()-1);
        Interval i2 = integral_t1_t_vy.intervalEvaluation(0,integral_t1_t_vy.size()-1);
        Interval i3 = hull(i1,i2);
        Box myBox(Interval(t1c*integral_t1_t_vy.dt,t1c*integral_t1_t_vy.dt + integral_t1_t_vy.size()*integral_t1_t_vy.dt),i3);
        plotIntegral->setInitialDomain(myBox);
        ////////////////////////////////////////////////////////////////////////////////////////
        //On n'est plus en train de selectionner une box, on veut afficher le tube position
        if (!plotPosition->boxList.empty()){
            plotPosition->newtonList.clear();
            plotPosition->boxList.clear();
            plotPosition->secondPointList.clear();
        }
        float startXPosition = pTubeX->at(t1c).center();
        float startYPosition = pTubeY->at(t1c).center();
        //        for(unsigned int i = 0 ; i < integral_t1_t_vx.size() ; i++){
        for(unsigned int i = t1c ; i < t2c ; i++){
            //plot Position => each boxes
            //myBox = Box(integral_t1_t_vy[i],integral_t1_t_vx[i]);
            //            myBox[1] = myBox[1] + startYPosition;
            //            myBox[2] = myBox[2] + startXPosition;
            //            myBox = Box(pTubeY->at(i),pTubeX->at(i));
            myBox = Box(pTubeX->at(i),pTubeY->at(i));
            myBox[1] = myBox[1];
            myBox[2] = myBox[2];
            plotPosition->addBox(i, myBox,positions);
        }
        plotPosition->addNewtonBox(myBox,Qt::red);

        for (unsigned int i = 0 ; i<= t1c ; i++){
            plotPosition->pointList[i].color  = Qt::black;
            plotPosition->pointList[i].width  = 1;
        }
        for (unsigned int i = t1c ; i<= t2c ; i++){
            plotPosition->pointList[i].color  = Qt::red;
            plotPosition->pointList[i].width  = 2;
        }
        for (unsigned int i = t2c ; i<= plotPosition->pointList.size()-1 ; i++){
            plotPosition->pointList[i].color  = Qt::black;
            plotPosition->pointList[i].width  = 1;
        }

        plotPosition->update();
    }
}
/**
  * mousePressEvent function intercept a click on tPlane
  * @param event a QMouseEvent
  */
void MainWindow::mousePressEvent(QMouseEvent * event)
{
    QPoint myPoint = ui->timeGraph->mapFromGlobal(event->globalPos());
    firstPoint.setX(plotTime->pixToX(myPoint.x()));
    firstPoint.setY(plotTime->pixToY(myPoint.y()));
    //Only catch click when loop detection has been performed
    if(ui->timeGraph->hasFocus() && tubesComputed){
        //Weare selecting a box
        if (event->button() == Qt::LeftButton &&
                (event->modifiers() == Qt::ControlModifier ||
                 event->modifiers() == Qt::ShiftModifier ||
                 zoomInActive || zoomOutActive || newtonToolBtActive))
        {
            selectingBox = true;
            movingPlane = false;
        }
        if (event->button() == Qt::RightButton)
        {
            selectingBox = false;
            movingPlane = true;
        }
    }
}
void MainWindow::mouseReleaseEvent(QMouseEvent * event)
{

    QPoint myPoint = ui->timeGraph->mapFromGlobal(event->globalPos());
    secondPoint.setX(plotTime->pixToX(myPoint.x()));
    secondPoint.setY(plotTime->pixToY(myPoint.y()));
    if(selectingBox && ui->timeGraph->hasFocus() && tubesComputed){
        //build the selected box
        Interval t1(floor(min(firstPoint.rx(),secondPoint.rx())),ceil(max(firstPoint.rx(),secondPoint.rx())));
        Interval t2(floor(min(firstPoint.ry(),secondPoint.ry())),ceil(max(firstPoint.ry(),secondPoint.ry())));
        Box T (t1,t2);
        if ((event->modifiers() == Qt::ControlModifier  || zoomInActive || zoomOutActive) && selectingBox){
            if(zoomOutActive){
                T = inflate(T,3);
                T = Inter(T,plotTime->getInitialDomain());
            }
            plotTime->setPriorDomain(T);
            selectingBox = false;
            T = Box();
            plotTime->plotNewtonTestedBox(T,NTested);
            ui->zoomInToolButton->setDown(false);
            zoomInActive = false;
            ui->zoomOutToolButton->setDown(false);
            zoomOutActive = false;
        }else if ((event->modifiers() == Qt::ShiftModifier || newtonToolBtActive) && selectingBox){
            ui->infoBox->insertPlainText("\n----------------------------------------------------------");
            ui->infoBox->insertPlainText("\nTesting t-box [t] = (["+QString::number(T[1].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+T[1].sup*pTubeDx->dt,'f',1)+"]["+QString::number(T[2].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+T[2].sup*pTubeDx->dt,'f',1)+"]");
            plotTime->plotNewtonTestedBox(T,NTested);
            plotTime->update();
            //If the box don't respect the condition t2 > t1, no need of Newton test
            Interval RMoins = Interval(-oo,0);
            if ((t2 - t1).isIn(RMoins) != ifalse){
                ui->infoBox->insertPlainText("\nLa boite testée ne respecte pas t2 > t1.");
            }else{
                //This box is now test by Newton
                Box N = myLoopDetector->newtonTest(T, *ui->infoBox);
                if ( !N.isEmpty() && N.isInInterior(T) == itrue ){//The Newton box is included in the selected t-box, it prove unicity and uniqueness
                    ui->infoBox->insertPlainText("\nNewton([t])");
                    ui->infoBox->insertPlainText("= ["+QString::number(N[1].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+N[1].sup*pTubeDx->dt,'f',1)+"]["+QString::number(N[2].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+N[2].sup*pTubeDx->dt,'f',1)+"] guarantee");
                    plotTime->addNewtonBox(N, NPassed);
                    plotTime->update();
                }else{ //if newton is not ok but included in the t-plane
                    Box intiDom = plotTime->getInitialDomain();
                    if (!N.isEmpty() && N.isIn(intiDom) == itrue){
                        ui->infoBox->insertPlainText("\nNewton(["+QString::number(T[1].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+T[1].sup*pTubeDx->dt,'f',1)+"]["+QString::number(T[2].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+T[2].sup*pTubeDx->dt,'f',1)+"])");
                        ui->infoBox->insertPlainText("= ["+QString::number(N[1].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+N[1].sup*pTubeDx->dt,'f',1)+"]["+QString::number(N[2].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+N[2].sup*pTubeDx->dt,'f',1)+"] NO guarantee");
                        plotTime->plotNewtonContractedBox(N,NResult);
                        plotTime->update();
                    }else{ //if Newton can't be computed (some times the jacobian can't be computed)
                        ui->infoBox->insertPlainText("\nLe resultat de newton ne peut pas etre affiché pour la boite testée.");
                        Box nullBox = Box();
                        plotTime->plotNewtonContractedBox(nullBox,NResult);
                        plotTime->update();
                    }
                }
            }
            selectingBox = false;
        }
    }else if(movingPlane && ui->timeGraph->hasFocus() && tubesComputed){

    }else if(ui->timeGraph->hasFocus() && tubesComputed){
        //On n'est plus en train de selectionner une box, on veut afficher le tube position
        if (!plotIntegral->tubeList.empty()){
            plotIntegral->tubeList.clear();
            plotPosition->newtonList.clear();
        }
        //        unsigned int t1c = secondPoint.x();
        //        unsigned int t2c = secondPoint.y();
        unsigned int t1c = min(secondPoint.x(),secondPoint.y());
        unsigned int t2c = max(secondPoint.x(),secondPoint.y());
        unsigned int step = 1 + (t2c- t1c)/100;
        ScalarTube integral_t1_t_vx = pTubeDx->timeIntegrationTube(t1c,t2c);
        ScalarTube integral_t1_t_vy = pTubeDy->timeIntegrationTube(t1c,t2c);
        //            ScalarTube integral_t1_t_vx = pTubeDx->timeIntegrationTubeLessData(t1c,t2c,step);
        //            ScalarTube integral_t1_t_vy = pTubeDy->timeIntegrationTubeLessData(t1c,t2c,step);
        plotIntegral->addTube(integral_t1_t_vx,pTubeDx->dt*t1c,Qt::blue);
        plotIntegral->addTube(integral_t1_t_vy,pTubeDy->dt*t1c,Qt::green);
        //Compute best axis for this plot
        Interval i1 = integral_t1_t_vx.intervalEvaluation(0,integral_t1_t_vx.size()-1);
        Interval i2 = integral_t1_t_vy.intervalEvaluation(0,integral_t1_t_vy.size()-1);
        Interval i3 = hull(i1,i2);
        Box myBox(Interval(t1c*integral_t1_t_vy.dt,t1c*integral_t1_t_vy.dt + integral_t1_t_vy.size()*integral_t1_t_vy.dt),i3);
        plotIntegral->setInitialDomain(myBox);
        ////////////////////////////////////////////////////////////////////////////////////////
        //On n'est plus en train de selectionner une box, on veut afficher le tube position
        if (!plotPosition->boxList.empty()){
            plotPosition->boxList.clear();
            plotPosition->secondPointList.clear();
        }
        float startXPosition = pTubeX->at(t1c).center();
        float startYPosition = pTubeY->at(t1c).center();
        //        for(unsigned int i = 0 ; i < integral_t1_t_vx.size() ; i++){
        //            //plot Position => each boxes
        //            Box myBox = Box(integral_t1_t_vy[i],integral_t1_t_vx[i]);
        //            myBox[1] = myBox[1] + startYPosition;
        //            myBox[2] = myBox[2] + startXPosition;
        //            plotPosition->addBox(i, myBox,positions);
        //        }
        //        plotPosition->addNewtonBox(plotPosition->boxList[plotPosition->boxList.size()-1].box,Qt::red);
        for(unsigned int i = t1c ; i < t2c ; i++){
            //plot Position => each boxes
            //            myBox = Box(pTubeY->at(i),pTubeX->at(i));
            myBox = Box(pTubeX->at(i),pTubeY->at(i));
            myBox[1] = myBox[1];
            myBox[2] = myBox[2];
            plotPosition->addBox(i, myBox,positions);
        }
        plotPosition->addNewtonBox(myBox,Qt::red);
        for (unsigned int i = 0 ; i<= t1c ; i++){
            plotPosition->pointList[i].color  = Qt::black;
            plotPosition->pointList[i].width  = 1;
        }
        for (unsigned int i = t1c ; i<= t2c ; i++){
            plotPosition->pointList[i].color  = Qt::red;
            plotPosition->pointList[i].width  = 2;
        }
        for (unsigned int i = t2c ; i<= plotPosition->pointList.size()-1 ; i++){
            plotPosition->pointList[i].color  = Qt::black;
            plotPosition->pointList[i].width  = 1;
        }
        plotPosition->update();
    }
    QScrollBar *sb = ui->infoBox->verticalScrollBar();
    sb->setValue(sb->maximum());
}

void MainWindow::keyPressEvent(QKeyEvent * event)
{
    if (tubesComputed && ui->positionGraph->hasFocus()){
        if (event->key() == Qt::Key_Plus){
            plotPosition->zoom(QCursor::pos(),1);
        }else if  (event->key() == Qt::Key_Minus){
            plotPosition->zoom(QCursor::pos(),-1);
        }
    }else if(tubesComputed && ui->timeGraph->hasFocus()){
        if (event->key() == Qt::Key_Plus){
            plotTime->zoom(QCursor::pos(),1);
        }else if  (event->key() == Qt::Key_Minus){
            plotTime->zoom(QCursor::pos(),-1);
        }
    }else if (tubesComputed && ui->IntegralGraph->hasFocus()){
        if (event->key() == Qt::Key_Plus){
            plotIntegral->zoom(QCursor::pos(),1);
        }else if (event->key() == Qt::Key_Minus){
            plotIntegral->zoom(QCursor::pos(),-1);
        }
    }
}

/**
  * setTrajFromFile function read a data file and put it in tubes
  * @param pTrajFile a string containing the name of the file
  * @param nbLines the number of lines you want to read in the file, default value = -1 to read all lines
  * @return true if the file has been successfully read, false if not
  */
bool MainWindow::setTrajFromFile(const std::string & pTrajFile, int nbLines)
{
    //ifstream Ctor open a file in read only mode
    std::ifstream fichier(pTrajFile.c_str());
    if ( fichier ) // false if file not opened
    {
        //vars in the file
        double t1, phi1, dphi1, theta1, dtheta1, psi1, dpsi1, vx1, dvx1, vy1, dvy1, vz1, dvz1, depth1, ddepth1, alt1, dalt1, x1, dx1, y1, dy1;
        // current line
        std::string ligne;
        //first loop, search the sample time
        int a = 1;
        while ( a ){
            std::getline( fichier, ligne );
            ligne = QString(ligne.c_str()).simplified().toStdString();
            a = ligne.compare(0,15,"% Sampling time");
        }
        //Sampling time found
        std::getline( fichier, ligne );
        sscanf(ligne.c_str(),"%lf",&t1);
        if (pTubeDxGlobal != NULL){
            delete pTubeDxGlobal;
            pTubeDxGlobal = NULL;
        }
        pTubeDxGlobal = new ScalarTube(t1);
        if (pTubeDyGlobal != NULL){
            delete pTubeDyGlobal;
            pTubeDyGlobal = NULL;
        }
        pTubeDyGlobal = new ScalarTube(t1);
        //second loop, search for interesting datas
        a = 1;
        while ( a ){
            std::getline( fichier, ligne );
            ligne = QString(ligne.c_str()).simplified().toStdString();
            a = ligne.compare(0,6,"%t phi");
        }//We found the commentary defining data order
        bool sortie = false;
        while (!sortie)
        {
            std::getline( fichier, ligne );
            if (ligne.find('$') != std::string::npos)
                sortie = true;
            else {
                sscanf(ligne.c_str(),"%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",&t1,&phi1,&dphi1,&theta1,&dtheta1,&psi1,&dpsi1,&vx1,&dvx1,&vy1,&dvy1,&vz1,&dvz1,&depth1,&ddepth1,&alt1,&dalt1,&x1,&dx1,&y1,&dy1);
                //we get speed in robot space, needed in world space
                Interval iPhi = phi1+Interval(-dphi1,dphi1);
                Interval iTheta = theta1+Interval(-dtheta1,dtheta1);
                Interval iPsi = psi1+Interval(-dpsi1,dpsi1);
                Interval vxr(vx1+Interval(-dvx1,dvx1));
                Interval vyr(vy1+Interval(-dvy1,dvy1));
                Interval vzr(vz1+Interval(-dvz1,dvz1));
                //transform
                pTubeDxGlobal->push_back(vxr*cos(iTheta)*cos(iPsi) - vyr*(cos(iPhi)*sin(iPsi)-sin(iTheta)*cos(iPsi)*sin(iPhi)) + vzr*(sin(iPhi)*sin(iPsi)+sin(iTheta)*cos(iPsi)*cos(iPhi)));
                pTubeDyGlobal->push_back(vxr*cos(iTheta)*sin(iPsi) + vyr*(cos(iPsi)*cos(iPhi)+sin(iTheta)*sin(iPsi)*sin(iPhi)) - vzr*(cos(iPsi)*sin(iPhi) - sin(iTheta)*cos(iPhi)*sin(iPsi)));
                //Only read what is asked
                if ((pTubeDxGlobal->size() >= nbLines) && (nbLines != -1))
                    return true;
            }
        }
        return true;
    }
    return false;
}

bool MainWindow::getKalmanResults(const std::string & pTrajFile)
{
    //ifstream Ctor open a file in read only mode
    std::ifstream fichier(pTrajFile.c_str());
    if ( fichier ) // false if file not opened
    {
        //vars in the file
        double xhat_x, xhat_y;
        // current line
        std::string ligne;
        bool sortie = false;
        while (!sortie)
        {
            std::getline( fichier, ligne );
            if (ligne.find('$') != std::string::npos)
                sortie = true;
            else {
                sscanf(ligne.c_str(),"%lf%lf",&xhat_x, &xhat_y);
                estimXGlobal.push_back(xhat_x);
                estimYGlobal.push_back(xhat_y);
            }
        }
        return true;
    }
    return false;
}

/**
  * cheatFramesResolution function is a dirty cheat for frames to fit in QGraphicsView
  */
void MainWindow::cheatFramesResolution(void)
{
    //Fix QGraphicsViews to be the larger square in the window
    //DIRTY
    //    QRect g = ui->timeGraph->geometry();
    //    int minSize = min(g.width(),g.height());
    //    ui->timeGraph->setGeometry(g.x(),g.y(),minSize,minSize);

    //    g = ui->positionGraph->geometry();
    //    minSize = min(g.width(),g.height());
    //    ui->positionGraph->setGeometry(g.x(),g.y(),minSize,minSize);

    //Frames got the same sizes as QGraphicsViews
    plotTime->setMyGeometry(ui->timeGraph->width(),ui->timeGraph->height());
    plotPosition->setMyGeometry(ui->positionGraph->width(),ui->positionGraph->height());
    plotIntegral->setMyGeometry(ui->IntegralGraph->width(),ui->IntegralGraph->height());
}
/**
  * resizeEvent function is an overloaded member and just call cheatFrameResolution
  * @param event an event
  */
void MainWindow::resizeEvent(QResizeEvent * event)
{
    cheatFramesResolution();
}
// PRIVATE SLOTS
void MainWindow::setColorMode(void)
{
    //Check color mode and adapt colors if needed
    if (ui->grayScaleChkBox->isChecked()){
        outside = Qt::white;
        inside = Qt::white;
        outside = outside.darker(105);
        inside = inside.darker(150);
        positions = outside;
        NTested = Qt::black;
        NTested = NTested.lighter(120);
        NResult = Qt::black;
        NResult = NResult.lighter(160);
        NPassed = Qt::black;
    }else{
        outside = QColor(204,227,255);
        positions = outside;
        positions.setAlpha(128);
        inside = QColor(255,84,81);
        NTested = Qt::blue;
        NResult = Qt::darkGreen;
        NPassed = Qt::black;
    }
}
void MainWindow::drawBoxesBorder(void)
{
    plotTime->setDrawRect(ui->drawRectChk->isChecked());
    plotPosition->setDrawRect(ui->drawRectChk->isChecked());
}
void MainWindow::drawBoxesCenter(void)
{
    pDrawBoxesCenter = ui->drawBoxCenterChkBox->isChecked();
    plotPosition->drawPointList = pDrawBoxesCenter;
    plotPosition->update();
}
void MainWindow::saveGraphs(void)
{
    //Save graphs with a unique name (by current time)
    QString time = QTime::currentTime().toString().remove(":");
    QString myFileName(QString("./savedImages/timeGraph_")+time+QString(".png"));
    plotTime->saveImage(myFileName.toStdString().c_str());
    myFileName = QString("./savedImages/positionGraph_")+time+QString(".png");
    plotPosition->saveImage(myFileName.toStdString().c_str());
}
void MainWindow::startLoopDetection(void)
{
    //NEW C.A. get kalman results
    computeKalmanTPlaneDist();
    //find loop button has been pressed
    if (tubesComputed)
        findLoop(1);
}
void MainWindow::drawDataClicked()
{
    QTime temps;
    temps.start();
    //draw button has been pressed
    clearAxes();
    //On prend en compte un possible sous-echantillonage pour kalman
    pTubeDx = new ScalarTube(ui->dataStepSpinBox->value()*pTubeDxGlobal->dt);
    pTubeDy = new ScalarTube(ui->dataStepSpinBox->value()*pTubeDxGlobal->dt);
    unsigned int iTube;
    int nbVal = std::min((int)estimXGlobal.size(), ui->nbPtRdrmrSpinBox->value());
    for (iTube = 0 ; iTube < nbVal ; iTube += ui->dataStepSpinBox->value()){
        estimX.push_back(estimXGlobal.at(iTube));
        estimY.push_back(estimYGlobal.at(iTube));
    }
    //On prend en compte un possible sous-echantillonage pour les tubes
    pTubeDx = new ScalarTube(ui->dataStepSpinBox->value()*pTubeDxGlobal->dt);
    pTubeDy = new ScalarTube(ui->dataStepSpinBox->value()*pTubeDxGlobal->dt);
    nbVal = std::min((int)pTubeDxGlobal->size(), ui->nbPtRdrmrSpinBox->value());
    for (iTube = 0 ; iTube < nbVal ; iTube += ui->dataStepSpinBox->value()){
        pTubeDx->push_back(pTubeDxGlobal->at(iTube));
        pTubeDy->push_back(pTubeDyGlobal->at(iTube));
    }
    //positions tubes are interval primitive of velocity tubes
    pTubeX = new ScalarTube(pTubeDx->timeIntegration());
    pTubeY = new ScalarTube(pTubeDy->timeIntegration());
    tubesComputed = true;
    cheatFramesResolution();
    //Draw read data
    drawDatas();
    //hide unsuable buttons
    ui->loopBt->setEnabled(true);
    ui->grayScaleChkBox->setEnabled(false);
    ui->infoBox->insertPlainText(QString::number(pTubeDx->size(),'d',0)+"/"+QString::number(pTubeDxGlobal->size(),'d',0)+" (1/"+QString::number(ui->dataStepSpinBox->value(),'d',0)+") Redermor datas drawn in "+QString::number(temps.elapsed())+" ms");
    QScrollBar *sb = ui->infoBox->verticalScrollBar();
    sb->setValue(sb->maximum());
}
//compute and draw LDKalman by euclidean distance
void MainWindow::computeKalmanTPlaneDist()
{
    computeKalmanTPlaneDistSeuilSeeDelay();
}
void MainWindow::computeKalmanTPlaneDistSeuilSeeDelay()
{
    QTime temps;
    temps.start();
    double seuil = ui->seuilDistKalmanStepSpinBox->value();
    for (uint ii=0;ii<estimX.size();ii++){
        for (uint jj=0;jj<estimX.size();jj++){
            double x = estimX[jj] - estimX[ii];
            double y = estimY[jj] - estimY[ii];
            double dist = sqrt(pow(x,2) + pow(y,2));
            dist=(dist>seuil)?seuil:dist;
        }
    }
    int timeEllapsedCompute = temps.elapsed();
    ui->infoBox->insertPlainText("\nLD via Kalman Filtering resolved in "+QString::number(timeEllapsedCompute)+" ms");
    temps.restart();
    for (uint ii=0;ii<estimX.size();ii++){
        for (uint jj=0;jj<estimX.size();jj++){
            double x = estimX[jj] - estimX[ii];
            double y = estimY[jj] - estimY[ii];
            double dist = sqrt(pow(x,2) + pow(y,2));
            dist=(dist>seuil)?seuil:dist;
            QColor cc=GetColour(seuil-dist,0,seuil);
            plotTime->addKalmanPoint(ii,jj,cc);
        }
    }
    plotTime->update();
    ui->infoBox->insertPlainText("\nLD via Kalman Filtering drawn in "+QString::number(temps.elapsed())+" ms for "+QString::number(plotTime->kalmanPointList.size(),'d',0)+" t-pairs");
    QScrollBar *sb = ui->infoBox->verticalScrollBar();
    sb->setValue(sb->maximum());
}

void MainWindow::resetAxes()
{
    //resest axes to the initial domains and clear graphs of Newton manual test boxes
    plotTime->newtonTestedBox.box = Box();
    plotTime->newtonContractedBox.box = Box();
    Box b = plotTime->getInitialDomain();
    plotTime->setPriorDomain(b);

    drawDatas();
    plotPosition->newtonList.clear();
    plotPosition->setDrawClickedBox(false);

    plotIntegral->tubeList.clear();
    b=plotIntegral->getInitialDomain();
    plotIntegral->setPriorDomain(b);
    //Plot Integral tubes of the entire mission
    plotIntegral->addTube(*pTubeX,0,Qt::blue);
    plotIntegral->addTube(*pTubeY,0,Qt::green);
    //Compute best axis for this plot
    Interval i1 = pTubeX->intervalEvaluation(0,pTubeX->size()-1);
    Interval i2 = pTubeY->intervalEvaluation(0,pTubeY->size()-1);
    Interval i3 = hull(i1,i2);
    Box myBox(Interval(0,(pTubeY->size()-1)*pTubeY->dt),i3);
    plotIntegral->setInitialDomain(myBox);
}
void MainWindow::clearAxes()
{
    ui->infoBox->clear();
    ui->saveButton->setEnabled(false);
    ui->loopBt->setEnabled(false);
    ui->grayScaleChkBox->setEnabled(true);
    //Clear axes
    plotTime->newtonTestedBox.box = Box();
    plotTime->newtonContractedBox.box = Box();
    plotTime->clear();
    plotPosition->clear();
    plotIntegral->clear();
    estimX.clear();
    estimY.clear();
    //Delete tubes if not empty
    if (pTubeX != NULL){
        delete pTubeX;
        pTubeX = NULL;
    }
    if (pTubeY != NULL){
        delete pTubeY;
        pTubeY = NULL;
    }
    if (pTubeDx != NULL){
        delete pTubeDx;
        pTubeDx = NULL;
    }
    if (pTubeDy != NULL){
        delete pTubeDy;
        pTubeDy = NULL;
    }
    if (myLoopDetector != NULL){
        delete myLoopDetector;
        myLoopDetector = NULL;
    }
    tubesComputed = false;
}

void MainWindow::on_drawDiagonalcheckBox_clicked()
{
    plotTime->setDrawDiagonal(ui->drawDiagonalcheckBox->isChecked());
    plotTime->update();
}

void MainWindow::on_zoomInToolButton_clicked()
{
    if (zoomInActive){
        ui->zoomInToolButton->setDown(false);
        zoomInActive = false;
    }else{
        ui->zoomInToolButton->setDown(true);
        zoomInActive = true;
        zoomOutActive = false;
        ui->zoomOutToolButton->setDown(false);
    }
}

void MainWindow::on_zoomOutToolButton_clicked()
{
    if (zoomOutActive){
        ui->zoomOutToolButton->setDown(false);
        zoomOutActive = false;
    }else{
        ui->zoomOutToolButton->setDown(true);
        zoomOutActive = true;
        zoomInActive = false;
        ui->zoomInToolButton->setDown(false);
    }
}

void MainWindow::on_zoomResetToolButton_clicked()
{
    zoomInActive = false;
    zoomOutActive = false;
    resetAxes();
}

void MainWindow::on_newtonCheckBox_clicked()
{
    if (ui->newtonCheckBox->isChecked()) {
        //Pour LUC JAULIN : la liste suivante est celle de toutes les boites que j'ai réussi a valider par Newton a la main. A décommenter pour qu'elles soient caculées automatiquement
        vector<Box > manualNewtonBoxes;
        manualNewtonBoxes.push_back(Box(Interval(26963,27139)/ui->dataStepSpinBox->value(),Interval(29674,29818)/ui->dataStepSpinBox->value()));
        manualNewtonBoxes.push_back(Box(Interval(10420,11321)/ui->dataStepSpinBox->value(),Interval(29228,30407)/ui->dataStepSpinBox->value()));//tb
        manualNewtonBoxes.push_back(Box(Interval(17124,17876)/ui->dataStepSpinBox->value(),Interval(25362,26355)/ui->dataStepSpinBox->value()));//tc
        manualNewtonBoxes.push_back(Box(Interval(13219,14224)/ui->dataStepSpinBox->value(),Interval(30196,31340)/ui->dataStepSpinBox->value()));
        manualNewtonBoxes.push_back(Box(Interval(20996,21759)/ui->dataStepSpinBox->value(),Interval(30542,31685)/ui->dataStepSpinBox->value()));
        manualNewtonBoxes.push_back(Box(Interval(48617,49073)/ui->dataStepSpinBox->value(),Interval(50896,51324)/ui->dataStepSpinBox->value()));
        manualNewtonBoxes.push_back(Box(Interval(10051,11283)/ui->dataStepSpinBox->value(),Interval(25916,27468)/ui->dataStepSpinBox->value()));
        manualNewtonBoxes.push_back(Box(Interval(17921,18671)/ui->dataStepSpinBox->value(),Interval(29946,30779)/ui->dataStepSpinBox->value()));
        manualNewtonBoxes.push_back(Box(Interval(43516,44092)/ui->dataStepSpinBox->value(),Interval(51928,52551)/ui->dataStepSpinBox->value()));
        manualNewtonBoxes.push_back(Box(Interval(40448,41171)/ui->dataStepSpinBox->value(),Interval(51377,52177)/ui->dataStepSpinBox->value()));
        manualNewtonBoxes.push_back(Box(Interval(36304,37270)/ui->dataStepSpinBox->value(),Interval(52297,53405)/ui->dataStepSpinBox->value()));
        manualNewtonBoxes.push_back(Box(Interval(25739,27050)/ui->dataStepSpinBox->value(),Interval(56609,57692)/ui->dataStepSpinBox->value()));
        manualNewtonBoxes.push_back(Box(Interval(20298,21640)/ui->dataStepSpinBox->value(),Interval(55211,56896)/ui->dataStepSpinBox->value()));
        manualNewtonBoxes.push_back(Box(Interval(29441,31950)/ui->dataStepSpinBox->value(),Interval(55308,57862)/ui->dataStepSpinBox->value()));//ta
        vector<Box >::iterator itN;
        vector<QColor > vColors;
        vColors.push_back(Qt::black);
        vColors.push_back(Qt::green);
        vColors.push_back(Qt::blue);
        vColors.push_back(Qt::cyan);
        vColors.push_back(Qt::magenta);
        vColors.push_back(Qt::yellow);
        vColors.push_back(Qt::darkMagenta);
        vColors.push_back(Qt::darkCyan);
        vColors.push_back(Qt::darkYellow);
        vColors.push_back(Qt::darkGreen);
        vColors.push_back(Qt::darkBlue);
        vColors.push_back(Qt::lightGray);
        vColors.push_back(Qt::white);
        vColors.push_back(Qt::red);
        Box domain = plotTime->getPriorDomain();
        vector<QColor >::iterator itColors = vColors.begin();
        for (itN = manualNewtonBoxes.begin() ; itN < manualNewtonBoxes.end() ; itN++){
            //Eeach box in the domain
            if ((*itN).isIn(domain) == itrue){
                //is tested by Newton
                Box N = myLoopDetector->newtonTest(*itN, *ui->infoBox, false);
                if ( !N.isEmpty() && N.isIn(*itN) == itrue ){
                    //draw newton box
                    //                plotTime->addBox(plotTime->boxList.count()+1,(*itN),Qt::blue);
                    plotTime->addNewtonBox(N, *itColors);
                    //print out
                    ui->infoBox->insertPlainText("\nNewton(["+QString::number((*itN)[1].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+(*itN)[1].sup*pTubeDx->dt,'f',1)+"]["+QString::number((*itN)[2].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+(*itN)[2].sup*pTubeDx->dt,'f',1)+"])");
                    ui->infoBox->insertPlainText("= ["+QString::number(N[1].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+N[1].sup*pTubeDx->dt,'f',1)+"]["+QString::number(N[2].inf*pTubeDx->dt,'f',1)+";"+QString::number(pTubeDx->dt+N[2].sup*pTubeDx->dt,'f',1)+"]");

                    //On va afficher, pour chaque NewtonBox, les positions(leur centre) de l'instant t1 à l'instant t2 en gris sur le trajet
                    unsigned int tchit = 50;
                    if ((*itN)[1].width() > tchit){
                        unsigned int id = (*itN)[1].centerInt();
                        unsigned int nbPos = 10;
                        for (unsigned int k=id-nbPos; k<=id+nbPos; k++){
                            //                            plotPosition->addBoldPoint(pTubeY->at(k).center(),pTubeX->at(k).center(),*itColors);
                            plotPosition->addBoldPoint(pTubeX->at(k).center(),pTubeY->at(k).center(),*itColors);
                        }
                    }else{
                        for (unsigned int k=(*itN)[1].inf; k<=(*itN)[1].sup; k++){
                            //                            plotPosition->addBoldPoint(pTubeY->at(k).center(),pTubeX->at(k).center(),*itColors);
                            plotPosition->addBoldPoint(pTubeX->at(k).center(),pTubeY->at(k).center(),*itColors);
                        }
                    }
                    if ((*itN)[2].width() > tchit){
                        unsigned int id = (*itN)[2].centerInt();
                        unsigned int nbPos = 10;
                        for (unsigned int k=k=id-nbPos; k<=id+nbPos; k++){
                            //                            plotPosition->addBoldPoint(pTubeY->at(k).center(),pTubeX->at(k).center(),*itColors);
                            plotPosition->addBoldPoint(pTubeX->at(k).center(),pTubeY->at(k).center(),*itColors);
                        }
                    }else{
                        for (unsigned int k=(*itN)[2].inf; k<=(*itN)[2].sup; k++){
                            //                            plotPosition->addBoldPoint(pTubeY->at(k).center(),pTubeX->at(k).center(),*itColors);
                            plotPosition->addBoldPoint(pTubeX->at(k).center(),pTubeY->at(k).center(),*itColors);
                        }
                    }

                }
                itColors++;
            }
        }
        ui->infoBox->insertPlainText("\n----------------------------------------------------------");
    }else{
        plotTime->newtonList.clear();
        plotPosition->boldPointList.clear();
    }
    plotTime->update();
    plotPosition->update();
    QScrollBar *sb = ui->infoBox->verticalScrollBar();
    sb->setValue(sb->maximum());
}

void MainWindow::on_newtonToolButton_clicked()
{
    if (newtonToolBtActive){
        ui->newtonToolButton->setDown(false);
        newtonToolBtActive = false;
    }else{
        ui->newtonToolButton->setDown(true);
        newtonToolBtActive = true;
    }
}

void MainWindow::on_kalmanCheckBox_clicked()
{
    plotTime->drawKalmanResults = ui->kalmanCheckBox->isChecked();
    plotTime->update();
}

void MainWindow::on_seuilDistKalmanStepSpinBox_editingFinished()
{
    plotTime->kalmanPointList.clear();
    computeKalmanTPlaneDist();
}

void MainWindow::on_seuilDistKalmanStepSpinBox_valueChanged(int arg1)
{
    //    estimX.clear();
    //    estimY.clear();
    //    computeKalmanTPlaneDist();
}

void MainWindow::on_plotEnsemblisteCheckBox_clicked()
{
    plotTime->drawLDResults = ui->plotEnsemblisteCheckBox->isChecked();
    plotTime->update();
}
