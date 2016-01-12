#include "loopdetector.h"

#include <QDebug>

LoopDetector::LoopDetector()
{
}
LoopDetector::LoopDetector(const double & pPrecision):precision(pPrecision)
{
}
LoopDetector::LoopDetector(const double & pPrecision, ScalarTube * tubeDx, ScalarTube * tubeDy, ScalarTube * tubeX, ScalarTube * tubeY):precision(pPrecision)
{
    pTubeDx = tubeDx;
    pTubeDy = tubeDy;
    intPrimDx = tubeX;
    intPrimDy = tubeY;
}
void LoopDetector::resolve(void)
{
    //Initialise research domain
    Box X(Interval(0,pTubeDx->size()-1),Interval(0,pTubeDx->size()-1));
    list<Box> L;
    L.push_back (X);
    //start the algorithm
    while ( !L.empty() )
    {
        X=L.front();
        L.pop_front();
        //test a box
        Iboolean test = isSolution(X);
        if (X.width() <= precision)//if the box is too small, put it in the T? set
            perhaps.push_back(X);
        else if ( test==itrue )//if the box is a solution, put it in the Tin set
            solutions.push_back(X);
        else if (test==ifalse)//if the box is not a solution, put it in the out set
            notSolutions.push_back(X);
        else {//if the box is unknown, bissect and put it in the list
            Box X1(2);  Box X2(2);
            BisectInteger(X,X1,X2);
            L.push_back(X1);L.push_back(X2);
        }
    }//End while ( !L.empty() )
}

Iboolean LoopDetector::isSolution(const Box & T)
{
    // Test 1 : integral test
    Interval timeTest = T[1] - T[2];
    Interval Rplus  = Interval(0,+oo);
    Interval Rmoins = Interval(-oo,0);
    Interval Ix,Iy;
    //Without SLAM data
    Ix = pTubeDx->boundedTimeIntegration(*intPrimDx,T[1].inf,T[1].sup,T[2].inf,T[2].sup);
    Iy = pTubeDy->boundedTimeIntegration(*intPrimDy,T[1].inf,T[1].sup,T[2].inf,T[2].sup);
    // Test 2 : partial injectivity test
    Interval dx = pTubeDx->fastIntervalEvaluation(T[1].inf,T[2].sup);
    Interval dy = pTubeDy->fastIntervalEvaluation(T[1].inf,T[2].sup);

    //Tout test
    if (timeTest.isIn(Rplus) == itrue || !Ix.contains(0) || !Iy.contains(0) || !dx.contains(0) || !dy.contains(0))
        return ifalse;

    //Test 3 : Inner test
    pair<Interval, Interval> IntegralDxInf_t1_t2 = pTubeDx->partialBoundedTimeIntegration(*intPrimDx,T[1].inf,T[1].sup,T[2].inf,T[2].sup);
    pair<Interval, Interval> IntegralDyInf_t1_t2 = pTubeDy->partialBoundedTimeIntegration(*intPrimDy,T[1].inf,T[1].sup,T[2].inf,T[2].sup);
    timeTest = T[1] - T[2];
    //Tin test
    if (timeTest.isIn(Rmoins) == itrue && IntegralDxInf_t1_t2.first.isIn(Rmoins) == itrue && IntegralDyInf_t1_t2.first.isIn(Rmoins) == itrue && IntegralDxInf_t1_t2.second.isIn(Rplus) == itrue && IntegralDyInf_t1_t2.second.isIn(Rplus) == itrue )
        return itrue;

    return iperhaps;
}

Box LoopDetector::newtonTest(const Box & T, QTextEdit & pTextEdit, bool printJ)
{
    Interval dx_t1 = pTubeDx->fastIntervalEvaluation(T[1].inf,T[1].sup);
    Interval dy_t1 = pTubeDy->fastIntervalEvaluation(T[1].inf,T[1].sup);
    Interval dx_t2 = pTubeDx->fastIntervalEvaluation(T[2].inf,T[2].sup);
    Interval dy_t2 = pTubeDy->fastIntervalEvaluation(T[2].inf,T[2].sup);
    Interval mdx_t1 = -pTubeDx->fastIntervalEvaluation(T[1].inf,T[1].sup);
    Interval mdx_t2 = -pTubeDx->fastIntervalEvaluation(T[2].inf,T[2].sup);
    //    qDebug()<<"\n\t-dx_t1 : [ "<<mdx_t1.inf<<" , "<<mdx_t1.sup<<"] \t dy_t1 : [ "<<dy_t1.inf<<" , "<<dy_t1.sup<<"]";
    //    qDebug()<<"\n\t-dx_t2 : [ "<<mdx_t2.inf<<" , "<<mdx_t2.sup<<"] \t dy_t2 : [ "<<dy_t2.inf<<" , "<<dy_t2.sup<<"]";
    if(printJ){
        pTextEdit.insertPlainText("\nJacobian : ");
        pTextEdit.insertPlainText("\n\t-Vx_t1 : [ "+QString::number(mdx_t1.inf)+" , "+QString::number(mdx_t1.sup)+"] \t Vy_t1 : [ "+QString::number(dy_t1.inf)+" , "+QString::number(dy_t1.sup)+"]");
        pTextEdit.insertPlainText("\n\t-Vx_t2 : [ "+QString::number(mdx_t2.inf)+" , "+QString::number(mdx_t2.sup)+"] \t Vy_t2 : [ "+QString::number(dy_t2.inf)+" , "+QString::number(dy_t2.sup)+"]");
    }
    //Compute the jacobian, if contains 0, we can't prove unicity
    Interval jacobien =  dy_t1 * dx_t2 - dx_t1 * dy_t2;
    if(printJ){
        pTextEdit.insertPlainText("\n\tdet(J)) : [ "+QString::number(jacobien.inf)+" , "+QString::number(jacobien.sup)+"]");
    }
    qDebug()<<"\njacobien : [ "<<jacobien.inf<<" , "<<jacobien.sup<<"]";
    //if the jacobian do not contains 0, it exist 0 or 1 solution in the t-box we're testing
    if (Interval(0).isIn(jacobien) != ifalse){
        qDebug()<<"Jacobian contains 0, Can't prove unicity";
        return Box();
    }

    //Choose a t1, t2 in [t1], [t2]
    int k1c = T[1].centerInt();
    int k2c = T[2].centerInt();
    //compute integral of velocity tubes from t1c to t2c
    Interval integralDx_t1c_t2c = pTubeDx->timeIntegration(k1c,k2c);
    Interval integralDy_t1c_t2c = pTubeDy->timeIntegration(k1c,k2c);
    //    qDebug()<<"\nintegralDx_t1c_t2c : [ "<<integralDx_t1c_t2c.inf<<" , "<<integralDx_t1c_t2c.sup<<"]";
    //    qDebug()<<"\nintegralDy_t1c_t2c : [ "<<integralDy_t1c_t2c.inf<<" , "<<integralDy_t1c_t2c.sup<<"]";
    //Compute Newton operator
    //    qDebug()<<"\nJacobian : [[ "<<a.inf<<" , "<<a.sup<<"] , [ "<<b.inf<<" , "<<b.sup<<"] ; [ "<<c.inf<<" , "<<c.sup<<"] , [ "<<d.inf<<" , "<<d.sup<<"] ]";
    Interval N_t1 = k1c*pTubeDx->dt - 1/jacobien * (dy_t2 * integralDx_t1c_t2c - dx_t2 * integralDy_t1c_t2c);
    Interval N_t2 = k2c*pTubeDx->dt - 1/jacobien * (dy_t1 * integralDx_t1c_t2c - dx_t1 * integralDy_t1c_t2c);
    //Transform the dicrete time values in tube indexes (to be coherent)
    Interval idN_T1,idN_T2;
    idN_T1.inf = pTubeDx->timeToIndex(N_t1.inf);
    idN_T1.sup = pTubeDx->timeToIndex(N_t1.sup);
    idN_T2.inf = pTubeDx->timeToIndex(N_t2.inf);
    idN_T2.sup = pTubeDx->timeToIndex(N_t2.sup);
    return Box(idN_T1,idN_T2);
}
Box LoopDetector::newtonTestMonoOcc(const Box & T)
{
    Interval dx_t1 = pTubeDx->fastIntervalEvaluation(T[1].inf,T[1].sup);
    Interval dy_t1 = pTubeDy->fastIntervalEvaluation(T[1].inf,T[1].sup);
    Interval dx_t2 = pTubeDx->fastIntervalEvaluation(T[2].inf,T[2].sup);
    Interval dy_t2 = pTubeDy->fastIntervalEvaluation(T[2].inf,T[2].sup);
    //Compute the jacobian, if contains 0, we can't prove unicity
    Interval jacobien =  dy_t1 * dx_t2 - dx_t1 * dy_t2;
    //if the jacobian do not contains 0, it exist 0 or 1 solution in the t-box we're testing
    if (Interval(0).isIn(jacobien) != ifalse){
        qDebug()<<"Jacobian contains 0, Can't prove unicity";
        return Box();
    }
    //Choose a t1, t2 in [t1], [t2]
    int k1c = T[1].centerInt();
    int k2c = T[2].centerInt();
    //compute integral of velocity tubes from t1c to t2c
    //Without SLAM data
    Interval integralDx_t1c_t2c = pTubeDx->timeIntegration(k1c,k2c);
    Interval integralDy_t1c_t2c = pTubeDy->timeIntegration(k1c,k2c);
    //Compute Newton operator without multiples occurences  in the inverse Jacobian
    Interval a = -dx_t1;
    Interval b = dx_t2;
    Interval c = -dy_t1;
    Interval d = dy_t2;

    Interval J11 = 1/(a -(b*c/d));
    Interval J12 = -1/((a*d/b)-c);
    Interval J21 = -1/((a*d/c)-b);
    Interval J22 = 1/(d -(b*c/a));

    //    qDebug()<<"Jacobian mono Occurence : [[ "<<J11.inf<<" , "<<J11.sup<<"] , [ "<<J12.inf<<" , "<<J12.sup<<"] ; [ "<<J21.inf<<" , "<<J21.sup<<"] , [ "<<J22.inf<<" , "<<J22.sup<<"] ]";

    Interval N_t1 = k1c*pTubeDx->dt - (integralDx_t1c_t2c/(a -(b*c/d))) + (integralDy_t1c_t2c/((a*d/b)-c));
    Interval N_t2 = k2c*pTubeDx->dt + (integralDx_t1c_t2c/((a*d/c)-b)) - (integralDy_t1c_t2c/(d -(b*c/a)));
    //Transform the dicrete time values in tube indexes (to be coherent)
    Interval idN_T1,idN_T2;
    idN_T1.inf = pTubeDx->timeToIndex(N_t1.inf);
    idN_T1.sup = pTubeDx->timeToIndex(N_t1.sup);
    idN_T2.inf = pTubeDx->timeToIndex(N_t2.inf);
    idN_T2.sup = pTubeDx->timeToIndex(N_t2.sup);
    return Box(idN_T1,idN_T2);
}
Box LoopDetector::topologicalDegreesTest(const Box & T, QTextEdit & pTextEdit, bool printJ)
{
    Interval dx_t1 = pTubeDx->fastIntervalEvaluation(T[1].inf,T[1].sup);
    Interval dy_t1 = pTubeDy->fastIntervalEvaluation(T[1].inf,T[1].sup);
    Interval dx_t2 = pTubeDx->fastIntervalEvaluation(T[2].inf,T[2].sup);
    Interval dy_t2 = pTubeDy->fastIntervalEvaluation(T[2].inf,T[2].sup);
    Interval mdx_t1 = -pTubeDx->fastIntervalEvaluation(T[1].inf,T[1].sup);
    Interval mdx_t2 = -pTubeDx->fastIntervalEvaluation(T[2].inf,T[2].sup);
    //    qDebug()<<"\n\t-dx_t1 : [ "<<mdx_t1.inf<<" , "<<mdx_t1.sup<<"] \t dy_t1 : [ "<<dy_t1.inf<<" , "<<dy_t1.sup<<"]";
    //    qDebug()<<"\n\t-dx_t2 : [ "<<mdx_t2.inf<<" , "<<mdx_t2.sup<<"] \t dy_t2 : [ "<<dy_t2.inf<<" , "<<dy_t2.sup<<"]";
    if(printJ){
        pTextEdit.insertPlainText("\nJacobian : ");
        pTextEdit.insertPlainText("\n\t-Vx_t1 : [ "+QString::number(mdx_t1.inf)+" , "+QString::number(mdx_t1.sup)+"] \t Vy_t1 : [ "+QString::number(dy_t1.inf)+" , "+QString::number(dy_t1.sup)+"]");
        pTextEdit.insertPlainText("\n\t-Vx_t2 : [ "+QString::number(mdx_t2.inf)+" , "+QString::number(mdx_t2.sup)+"] \t Vy_t2 : [ "+QString::number(dy_t2.inf)+" , "+QString::number(dy_t2.sup)+"]");
    }
    //Compute the jacobian, if contains 0, we can't prove unicity
    Interval jacobien =  dy_t1 * dx_t2 - dx_t1 * dy_t2;
    if(printJ){
        pTextEdit.insertPlainText("\n\tdet(J)) : [ "+QString::number(jacobien.inf)+" , "+QString::number(jacobien.sup)+"]");
    }
    qDebug()<<"\njacobien : [ "<<jacobien.inf<<" , "<<jacobien.sup<<"]";
    //if the jacobian do not contains 0, it exist 0 or 1 solution in the t-box we're testing
    if (Interval(0).isIn(jacobien) != ifalse){
        qDebug()<<"Jacobian contains 0, Can't prove unicity";
        return Box();
    }

    //Choose a t1, t2 in [t1], [t2]
    int k1c = T[1].centerInt();
    int k2c = T[2].centerInt();
    //compute integral of velocity tubes from t1c to t2c
    Interval integralDx_t1c_t2c = pTubeDx->timeIntegration(k1c,k2c);
    Interval integralDy_t1c_t2c = pTubeDy->timeIntegration(k1c,k2c);
    //    qDebug()<<"\nintegralDx_t1c_t2c : [ "<<integralDx_t1c_t2c.inf<<" , "<<integralDx_t1c_t2c.sup<<"]";
    //    qDebug()<<"\nintegralDy_t1c_t2c : [ "<<integralDy_t1c_t2c.inf<<" , "<<integralDy_t1c_t2c.sup<<"]";
    //Compute Newton operator
    //    qDebug()<<"\nJacobian : [[ "<<a.inf<<" , "<<a.sup<<"] , [ "<<b.inf<<" , "<<b.sup<<"] ; [ "<<c.inf<<" , "<<c.sup<<"] , [ "<<d.inf<<" , "<<d.sup<<"] ]";
    Interval N_t1 = k1c*pTubeDx->dt - 1/jacobien * (dy_t2 * integralDx_t1c_t2c - dx_t2 * integralDy_t1c_t2c);
    Interval N_t2 = k2c*pTubeDx->dt - 1/jacobien * (dy_t1 * integralDx_t1c_t2c - dx_t1 * integralDy_t1c_t2c);
    //Transform the dicrete time values in tube indexes (to be coherent)
    Interval idN_T1,idN_T2;
    idN_T1.inf = pTubeDx->timeToIndex(N_t1.inf);
    idN_T1.sup = pTubeDx->timeToIndex(N_t1.sup);
    idN_T2.inf = pTubeDx->timeToIndex(N_t2.inf);
    idN_T2.sup = pTubeDx->timeToIndex(N_t2.sup);
    return Box(idN_T1,idN_T2);
}
