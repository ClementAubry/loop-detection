#ifndef SCALARTUBE_H
#define SCALARTUBE_H

#include "interval.h"
#include <vector>
#include <map>

using std::vector;
using std::map;
using std::pair;

class ScalarTube : public vector<Interval>
{
public:
    // ACCESSIBLES MEMBERS
    double dt;

    // Constructors
    ScalarTube(void);
    ScalarTube(const double);
    ScalarTube(const vector<Interval>&, double);
    ScalarTube(const Interval&,int, double);
    ScalarTube(const ScalarTube&);

    // Destructor.
    ~ScalarTube();

    // MEMBERS, OVERLOAD (unary operator)
    void operator=(const ScalarTube&);

    Interval                  intervalEvaluation(unsigned int, unsigned int);
    Interval                  fastIntervalEvaluation(unsigned int, unsigned int);
    ScalarTube            	  timeIntegration(void);
    ScalarTube                timeIntegrationTube(unsigned  int,unsigned  int);
    ScalarTube                timeIntegrationTubeLessData(unsigned  int,unsigned  int, unsigned int);
    Interval                  timeIntegration(unsigned  int,unsigned  int);
    Interval                  boundedTimeIntegration(ScalarTube&,unsigned  int,unsigned  int,unsigned  int,unsigned  int);
    Interval                  fastBoundedTimeIntegration(ScalarTube&,unsigned  int,unsigned  int,unsigned  int,unsigned  int);
    pair<Interval, Interval > partialBoundedTimeIntegration(ScalarTube & , unsigned  int , unsigned  int , unsigned  int , unsigned  int );

    unsigned int              timeToIndex(double pTime);

    // FRIENDS
    friend ScalarTube operator+  (const ScalarTube&, const ScalarTube&);
    friend ScalarTube operator+  (const ScalarTube&, const double&);
    friend ScalarTube operator+  (const double&, const ScalarTube&);
    friend ScalarTube operator+  (const ScalarTube&, const Interval&);
    friend ScalarTube operator+  (const Interval&, const ScalarTube&);

    friend ScalarTube operator-  (const ScalarTube&);
    friend ScalarTube operator-  (const ScalarTube&, const ScalarTube&);
    friend ScalarTube operator-  (const ScalarTube&, const double&);
    friend ScalarTube operator-  (const double&, const ScalarTube&);

    friend ScalarTube operator*  (const ScalarTube&, const double&);
    friend ScalarTube operator*  (const double&, const ScalarTube&);


    friend double     maxNotoo(ScalarTube&);
    friend double     minNotoo(ScalarTube&);
    friend double     max(ScalarTube&);
    friend double     min(ScalarTube&);

    friend ScalarTube inter(ScalarTube&,ScalarTube&); // Intersection
    friend ScalarTube&   operator&  (const ScalarTube&,const ScalarTube&);  // Intersection
    friend ScalarTube&   operator&  (const ScalarTube&,const Interval&);  // Intersection

private:
    //For Fast interval evaluation with min/max method.
    //Also for fast integral evaluation using a similar method
    map<unsigned int,double> fInfMin;
    map<unsigned int,double> fInfMax;
    map<unsigned int,double> fSupMin;
    map<unsigned int,double> fSupMax;

    //Private functions
    bool computeMinMax(void); //Compute local min and max of the lower/upper bounds of the tube

};
#endif // SCALARTUBE_H
