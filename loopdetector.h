#ifndef LOOPDETECTOR_H
#define LOOPDETECTOR_H

#include <QTextEdit>
#include <list>
#include "interval/iboolean.h"
#include "interval/interval.h"
#include "interval/box.h"
#include "interval/scalarTube.h"

using std::list;
using std::pair;

class LoopDetector
{
public:
    //Ctors.
    LoopDetector();
    LoopDetector(const double & );
    LoopDetector(const double & , ScalarTube *, ScalarTube *, ScalarTube *, ScalarTube *);
    Box newtonTest(const Box & T, QTextEdit &pTextEdit, bool printJ = true);
    Box newtonTestMonoOcc(const Box & T);

    //Methods
    void resolve(void);
    inline void clear(void){
        perhaps.clear();
        solutions.clear();
        notSolutions.clear();
        newton.clear();
    }
    //Getters/Setters
    inline double getPrecision(void) const {
        return precision;
    }
    inline void setPrecision(const double & pPrecision) {
        precision = pPrecision;
    }
    inline list<Box> getSolutions(void) const {
        return solutions;
    }
    inline list<Box> getNotSolutions(void) const {
        return notSolutions;
    }
    inline list<Box> getPerhaps(void) const {
        return perhaps;
    }
    inline list<Box> getNewton(void) const {
        return newton;
    }
    list<pair<Box,Box> > getContracted(void) const {
        return contracted;
    }

protected:
    Iboolean isSolution(const Box & T);

private :
    //For resolution
    double precision;

    //INPUT
    ScalarTube * pTubeDx;
    ScalarTube * pTubeDy;
    ScalarTube * intPrimDx;
    ScalarTube * intPrimDy;

    //OUTPUT
    list<Box> solutions;
    list<Box> notSolutions;
    list<Box> perhaps;
    list<Box> newton;
    list<pair<Box,Box> > contracted;


};

#endif // LOOPDETECTOR_H
