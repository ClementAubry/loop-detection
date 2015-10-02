#include "scalarTube.h"
#include <algorithm>
#include <QDebug>

// Constructors
ScalarTube::ScalarTube():dt(0.0)
{
  this->clear();
}
ScalarTube::ScalarTube(const double pDt):dt(pDt)
{
  this->clear();
}
ScalarTube::ScalarTube(const vector<Interval> & pVect, double pDt):dt(pDt)
{
  this->clear();
  this->assign(pVect.begin(),pVect.end());
}
ScalarTube::ScalarTube(const Interval & pInt,int nbElem, double pDt):dt(pDt)
{
  this->clear();
  this->assign(nbElem,pInt);
}
ScalarTube::ScalarTube(const ScalarTube & pTube) : vector<Interval>(pTube), dt(pTube.dt)
{}
// Desctuctor
ScalarTube::~ScalarTube()
{}

// MEMBERS
void ScalarTube::operator=(const ScalarTube & pTube)
{
  this->clear();
  this->assign(pTube.begin(),pTube.end());
  this->dt=pTube.dt;
}
Interval ScalarTube::intervalEvaluation(unsigned int kmin, unsigned int kmax)
{
  Interval I((*this)[kmin]);
  for (unsigned int i = kmin+1 ; i <= kmax ; i++){
    if ( (*this)[i].sup > I.sup )
      I.sup = (*this)[i].sup;
    if ( (*this)[i].inf < I.inf)
      I.inf = (*this)[i].inf;
  }
  return I;
}
Interval ScalarTube::fastIntervalEvaluation(unsigned int k1, unsigned int k2)
{
  if (fInfMin.size() < 2)
    computeMinMax();
  Interval I;
  I.inf = std::min((*this)[k1].inf,(*this)[k2].inf);
  I.sup = std::max((*this)[k1].sup,(*this)[k2].sup);
  map<unsigned int,double>::iterator it,itlow,itup;
  //First, search lower bound
  itlow=fInfMin.lower_bound (k1);
  itup=fInfMin.lower_bound (k2);
  for (it = itlow ; it != itup ; it++){
    if(it->second < I.inf)
      I.inf = it->second;
  }
  //Now, search upper bound
  itlow=fSupMax.lower_bound (k1);
  itup=fSupMax.lower_bound (k2);
  for (it = itlow ; it != itup ; it++){
    if(it->second > I.sup)
      I.sup = it->second;
  }
  return I;
}
Interval ScalarTube::timeIntegration(unsigned  int kmin, unsigned  int kmax)
{
  double ssup = 0.0;
  double sinf = 0.0;
  for (unsigned int k=kmin; k<=kmax; k++){
      ssup+=(*this)[k].sup*this->dt;
      sinf+=(*this)[k].inf*this->dt;
  }
  return Interval(sinf,ssup);
}
ScalarTube ScalarTube::timeIntegration(void)
{
  ScalarTube u(this->dt);
  for (unsigned int k=0;k<this->size();k++){
      u.push_back(this->timeIntegration(0,k));
  }
  return u;
}
ScalarTube ScalarTube::timeIntegrationTube(unsigned  int kmin, unsigned  int kmax)
{
  ScalarTube u(this->dt);
  for (unsigned int k=kmin;k<kmax;k++){
      u.push_back(this->timeIntegration(kmin,k));
  }
  return u;
}
ScalarTube ScalarTube::timeIntegrationTubeLessData(unsigned  int kmin, unsigned  int kmax, unsigned int step)
{
  ScalarTube u(this->dt);
  for (unsigned int k=kmin;k<kmax;k+=step){
      u.push_back(this->timeIntegration(kmin,k));
  }
  return u;
}
Interval ScalarTube::boundedTimeIntegration(ScalarTube & iP, unsigned  int k1min, unsigned  int k1max, unsigned  int k2min, unsigned  int k2max)
{
  //@ param iP for interval primitive
  //search yMin_t1
  Interval yMin_t1(iP[k1min].inf,iP[k1min].inf);
  for (unsigned int i = k1min+1 ; i <= k1max ; i++) {
    if (iP[i].inf < yMin_t1.inf)
      yMin_t1.inf = iP[i].inf;
    if (iP[i].inf > yMin_t1.sup)
      yMin_t1.sup = iP[i].inf;
  }
  //search yMax_t1
  Interval yMax_t1(iP[k1min].sup,iP[k1min].sup);
  for (unsigned int i = k1min+1 ; i <= k1max ; i++) {
    if (iP[i].sup < yMax_t1.inf)
      yMax_t1.inf = iP[i].sup;
    if (iP[i].sup > yMax_t1.sup)
      yMax_t1.sup = iP[i].sup;
  }
  //search yMin_t2
  Interval yMin_t2(iP[k2min].inf,iP[k2min].inf);
  for (unsigned int i = k2min+1 ; i <= k2max ; i++) {
    if (iP[i].inf < yMin_t2.inf)
      yMin_t2.inf = iP[i].inf;
    if (iP[i].inf > yMin_t2.sup)
      yMin_t2.sup = iP[i].inf;
  }
  //search yMax_t2
  Interval yMax_t2(iP[k2min].sup,iP[k2min].sup);
  for (unsigned int i = k2min+1 ; i <= k2max ; i++) {
    if (iP[i].sup < yMax_t2.inf)
      yMax_t2.inf = iP[i].sup;
    if (iP[i].sup > yMax_t2.sup)
      yMax_t2.sup = iP[i].sup;
  }
  //Compute integral with interval bounds
  return Interval((yMin_t2 - yMin_t1).inf, (yMax_t2 - yMax_t1).sup);
}
Interval ScalarTube::fastBoundedTimeIntegration(ScalarTube & iP, unsigned  int k1min, unsigned  int k1max, unsigned  int k2min, unsigned  int k2max)
{
  //@ param iP for interval primitive
  if (fInfMin.size() < 2)
    computeMinMax();

  //search yMin_t1
  Interval yMin_t1;
  yMin_t1.inf = std::min(iP[k1min].inf,iP[k1max].inf);
  yMin_t1.sup = std::max(iP[k1min].inf,iP[k1max].inf);
  map<unsigned int,double>::iterator it,itlow,itup;
  //First, search lower bound
  itlow=fInfMin.lower_bound (k1min);
  itup=fInfMin.upper_bound (k1max);
  for (it = itlow ; it != itup ; it++){
    if(it->second < yMin_t1.inf)
      yMin_t1.inf = it->second;
  }
  //Second, search upper bound
  itlow=fInfMax.lower_bound (k1min);
  itup=fInfMax.upper_bound (k1max);
  for (it = itlow ; it != itup ; it++){
    if(it->second > yMin_t1.sup)
      yMin_t1.sup = it->second;
  }
  //search yMax_t1
  Interval yMax_t1;
  yMax_t1.inf = std::min(iP[k1min].sup,iP[k1max].sup);
  yMax_t1.sup = std::max(iP[k1min].sup,iP[k1max].sup);
  //First, search lower bound
  itlow=fSupMin.lower_bound (k1min);
  itup=fSupMin.upper_bound (k1max);
  for (it = itlow ; it != itup ; it++){
    if(it->second < yMax_t1.inf)
      yMax_t1.inf = it->second;
  }
  //Second, search upper bound
  itlow=fSupMax.lower_bound (k1min);
  itup=fSupMax.upper_bound (k1max);
  for (it = itlow ; it != itup ; it++){
    if(it->second > yMax_t1.sup)
      yMax_t1.sup = it->second;
  }
  //search yMin_t2
  Interval yMin_t2;
  yMin_t2.inf = std::min(iP[k2min].inf,iP[k2max].inf);
  yMin_t2.sup = std::max(iP[k2min].inf,iP[k2max].inf);
  //First, search lower bound
  itlow=fInfMin.lower_bound (k2min);
  itup=fInfMin.upper_bound (k2max);
  for (it = itlow ; it != itup ; it++){
    if(it->second < yMin_t2.inf)
      yMin_t2.inf = it->second;
  }
  //Second, search upper bound
  itlow=fInfMax.lower_bound (k2min);
  itup=fInfMax.upper_bound (k2max);
  for (it = itlow ; it != itup ; it++){
    if(it->second > yMin_t2.sup)
      yMin_t2.sup = it->second;
  }
  //search yMax_t2
  Interval yMax_t2;
  yMax_t2.inf = std::min(iP[k2min].sup,iP[k2max].sup);
  yMax_t2.sup = std::max(iP[k2min].sup,iP[k2max].sup);
  //First, search lower bound
  itlow=fSupMin.lower_bound (k2min);
  itup=fSupMin.upper_bound (k2max);
  for (it = itlow ; it != itup ; it++){
    if(it->second < yMax_t2.inf)
      yMax_t2.inf = it->second;
  }
  //Second, search upper bound
  itlow=fSupMax.lower_bound (k2min);
  itup=fSupMax.upper_bound (k2max);
  for (it = itlow ; it != itup ; it++){
    if(it->second > yMax_t2.sup)
      yMax_t2.sup = it->second;
  }
  //Compute integral with interval bounds
  return Interval((yMin_t2 - yMin_t1).inf, (yMax_t2 - yMax_t1).sup);
}
pair<Interval, Interval > ScalarTube::partialBoundedTimeIntegration(ScalarTube & iP, unsigned  int k1min, unsigned  int k1max, unsigned  int k2min, unsigned  int k2max)
{
  //@ param iP for interval primitive
  //search yMin_t1
  Interval yMin_t1(iP[k1min].inf,iP[k1min].inf);
  for (unsigned int i = k1min+1 ; i <= k1max ; i++) {
    if (iP[i].inf < yMin_t1.inf)
      yMin_t1.inf = iP[i].inf;
    if (iP[i].inf > yMin_t1.sup)
      yMin_t1.sup = iP[i].inf;
  }
  //search yMax_t1
  Interval yMax_t1(iP[k1min].sup,iP[k1min].sup);
  for (unsigned int i = k1min+1 ; i <= k1max ; i++) {
    if (iP[i].sup < yMax_t1.inf)
      yMax_t1.inf = iP[i].sup;
    if (iP[i].sup > yMax_t1.sup)
      yMax_t1.sup = iP[i].sup;
  }
  //search yMin_t2
  Interval yMin_t2(iP[k2min].inf,iP[k2min].inf);
  for (unsigned int i = k2min+1 ; i <= k2max ; i++) {
    if (iP[i].inf < yMin_t2.inf)
      yMin_t2.inf = iP[i].inf;
    if (iP[i].inf > yMin_t2.sup)
      yMin_t2.sup = iP[i].inf;
  }
  //search yMax_t2
  Interval yMax_t2(iP[k2min].sup,iP[k2min].sup);
  for (unsigned int i = k2min+1 ; i <= k2max ; i++) {
    if (iP[i].sup < yMax_t2.inf)
      yMax_t2.inf = iP[i].sup;
    if (iP[i].sup > yMax_t2.sup)
      yMax_t2.sup = iP[i].sup;
  }
  //Compute integral with interval bounds
  Interval bdIntegral_Ymin = yMin_t2 - yMin_t1;
  Interval bdIntegral_Ymax = yMax_t2 - yMax_t1;
  return pair<Interval, Interval >(bdIntegral_Ymin,bdIntegral_Ymax);
}
unsigned int ScalarTube::timeToIndex(double pTime)
{
  unsigned int index = static_cast<unsigned int>(pTime/dt);
  if (index > this->size()-1)
    return this->size()-1;
  else
    return index;
}
// FRIENDS
ScalarTube operator+  (const ScalarTube & t1, const ScalarTube & t2)
{
  if ( (t1.size()!=t2.size()) || (t1.dt!=t2.dt))
    return ScalarTube();
  ScalarTube r(t1);
  for (unsigned int i=0; i<t1.size(); i++)
    r[i] =  r[i]+t2[i];
  return r;
}
ScalarTube operator+  (const ScalarTube & t, const double & val)
{
  ScalarTube r(t);
  for (unsigned int i=0; i<t.size(); i++)
      r[i] =  r[i]+val;
  return r;
}
ScalarTube operator+  (const double & val, const ScalarTube & t)
{
  ScalarTube r(t);
  for (unsigned int i=0; i<t.size(); i++)
      r[i] =  r[i]+val;
  return r;
}
ScalarTube operator+  (const ScalarTube & t, const Interval & iVal)
{
  ScalarTube r(t);
  for (unsigned int i=0; i<t.size(); i++)
      r[i] =  r[i]+iVal;
  return r;
}
ScalarTube operator+  (const Interval & iVal, const ScalarTube & t)
{
  ScalarTube r(t);
  for (unsigned int i=0; i<t.size(); i++)
      r[i] =  r[i]+iVal;
  return r;
}

ScalarTube operator-  (const ScalarTube & t)
{
  ScalarTube r(t);
  for (unsigned int i=0; i<t.size(); i++)
    r[i] = -r[i];
  return r;
}
ScalarTube operator-  (const ScalarTube & t1, const ScalarTube & t2)
{
  if ( (t1.size()!=t2.size()) || (t1.dt!=t2.dt))
    return ScalarTube();
  ScalarTube r(t1);
  for (unsigned int i=0; i<t1.size(); i++)
    r[i] = r[i]-t2[i];
  return r;
}
ScalarTube operator-  (const ScalarTube & t, const double & val)
{
  ScalarTube r(t);
  for (unsigned int i=0; i<t.size(); i++)
    r[i] = r[i]-val;
  return r;
}
ScalarTube operator-  (const double&val, const ScalarTube&t)
{
  ScalarTube r(Interval(),t.size(),t.dt);
  for (unsigned int i=0; i<t.size(); i++)
    r[i] = val - t[i];
  return r;
}

ScalarTube operator*  (const ScalarTube & b, const double & a)
{
  ScalarTube r(b.dt);
  for (unsigned int i=0; i<b.size(); i++)
    r.push_back(a*b[i]);
  return r;
}
ScalarTube operator*  (const double & a, const ScalarTube & b)
{
  ScalarTube r(b.dt);
  for (unsigned int i=0; i<b.size(); i++)
    r.push_back(a*b[i]);
  return r;
}

double maxNotoo(ScalarTube & t)
{
  double r=-oo;
  for (unsigned int i=0; i<t.size(); i++){
      double bsup=t[i].sup;
      if ((r< bsup) && (bsup!=+oo))
          r=t[i].sup;
  }
  if (!std::isinf(r))
      return r;
  else
      return NAN;
}
double minNotoo(ScalarTube & t)
{
  double r=+oo;
  for (unsigned int i=0; i<t.size(); i++){
      double binf=t[i].inf;
      if ((r>binf) && (binf!=-oo))
          r=t[i].inf;
  }
  if (!std::isinf(r))
      return r;
  else
      return NAN;
}
double max(ScalarTube & t)
{
  double r=-oo;
  for (unsigned int i=0; i<t.size(); i++){
      double bsup=t[i].sup;
      if (r< bsup)
          r=t[i].sup;
  }
  return r;
}
double min(ScalarTube & t)
{
  double r=+oo;
  for (unsigned int i=0; i<t.size(); i++){
      double binf=t[i].inf;
      if (r>binf)
          r=t[i].inf;
  }
  return r;
}
//intersections
ScalarTube inter(ScalarTube & t,ScalarTube & u)
{
    ScalarTube *r = new ScalarTube();
    if ( (t.size()!=u.size()) || (t.dt!=u.dt) )
        return *r;
    r->dt=t.dt;
    for (unsigned int i=0; i<t.size(); i++){
        r->push_back(t[i]&u[i]);
    }
    return *r;
}
ScalarTube&	operator& (const ScalarTube& a,const ScalarTube& b)
{
    ScalarTube *r = new ScalarTube();
    if ( (a.size()!=b.size()) || (a.dt!=b.dt) )
        return *r;
    r->dt=a.dt;
    for (unsigned int i=0; i<a.size(); i++){
        r->push_back((a[i]&b[i]));
    }
    return *r;
}
ScalarTube&	operator& (const ScalarTube& a,const Interval& b)
{
    ScalarTube *r = new ScalarTube(a.dt);
    for (unsigned int i=0; i<a.size(); i++)
        r->push_back((a[i]&b));
    return *r;
}
bool ScalarTube::computeMinMax(void)
{
  double ancValInf = (*this)[0].inf;
  double ancValSup = (*this)[0].sup;
  fInfMin[0] = (*this)[0].inf;
  fInfMax[0] = (*this)[0].inf;
  fSupMin[0] = (*this)[0].sup;
  fSupMax[0] = (*this)[0].sup;
  size_t lastIndex = this->size()-1;
  for (unsigned int i=1 ; i < lastIndex ; i++){
    //build local min and max maps of the lower boud of the tube
    if ((*this)[i].inf < ancValInf && (*this)[i].inf <= (*this)[i+1].inf)
      fInfMin[i] = (*this)[i].inf;
    if ((*this)[i].inf > ancValInf && (*this)[i].inf >= (*this)[i+1].inf)
      fInfMax[i] = (*this)[i].inf;
    //build local min and max maps of the upper boud of the tube
    if ((*this)[i].sup < ancValSup && (*this)[i].sup <= (*this)[i+1].sup)
      fSupMin[i] = (*this)[i].sup;
    if ((*this)[i].sup > ancValSup && (*this)[i].sup >= (*this)[i+1].sup)
      fSupMax[i] = (*this)[i].sup;
    ancValInf = (*this)[i].inf;
    ancValSup = (*this)[i].sup;
  }
  fInfMin[lastIndex] = (*this)[lastIndex].inf;
  fInfMax[lastIndex] = (*this)[lastIndex].inf;
  fSupMin[lastIndex] = (*this)[lastIndex].sup;
  fSupMax[lastIndex] = (*this)[lastIndex].sup;
  return true;
}
