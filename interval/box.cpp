#include "box.h"
#include <cmath>
using namespace std;

// Constructors
Box::Box ()
{
  this->dim=0;
  this->data=NULL;
}
Box::Box (int dimension)
{
  this->dim=dimension;
  this->data=new Interval[dim];
  Interval empty(0);
  for (int i=0;i<=this->dim-1;i++) this->data[i]=empty;
}
Box::Box (Interval a, Interval b)
{
  this->dim=2;
  this->data=new Interval[2];
  this->operator [](1)=a;
  this->operator [](2)=b;
}
Box::Box (Interval a, Interval b, Interval c)
{
  this->dim=3;
  this->data=new Interval[3];
  this->data[0]=a;
  this->data[1]=b;
  this->data[2]=c;
}
Box::Box (const Box &v)
{
  if (&v == this) return;
  this->dim=v.dim;
  this->data=new Interval[this->dim];
  for (int k=1; k<=this->dim; k++) this->operator [](k)=v[k];
}
// Destructor
Box::~Box () {delete [] data;}


// Members Overloaded Operators
Interval& Box::operator[] (int i)  const
{
  return this->data[i-1];
}
Box& Box::operator=(const Box &v)
{
  this->dim=v.dim;
  delete [] this->data;
  this->data=new Interval[this->dim];
  for (int k=1; k<=this->dim; k++) this->operator [](k)=v[k];
  return *this;
}
// Others Overloaded Operators
bool operator==(const Box& v,const Box& w)
{
  if (v.dim != w.dim) return false;
  for (int i = 1 ; i <= v.dim ; i++){
    if (v[i] != w[i])
      return false;
  }
  return true;
}
Box operator+(const Box& v,const Box& w)
{
  Box r(v.dim);
  for (int k=1; k<=r.dim; k++) r[k]=v[k]+w[k];
  return (r);
}
Box operator-(const Box& v)
{
  Box r(v.dim);
  for (int k=1; k<=r.dim; k++) r[k]=-v[k];
  return (r);
}
Box operator-(const Box& v,const Box& w)
{
  return (v+(-w));
}
Box operator*(const Interval& a,const Box& v)
{
  Box r(v.dim);
  for (int k=1; k<=r.dim; k++) r[k]=a*v[k];
  return (r);
}
Box	operator*(const Box& v,const Interval& a)
{
  return (a*v);
}
ostream& operator<<(ostream& os, const Box& v)
{
  cout << "Box \n";
  cout << "   dim="<<v.dim << endl;
  if (v.isEmpty()) os<<"Not a Box !"<< endl;
  for (int i=1; i<=v.dim; i++)
    os << "  " << i << ": "<< v[i] << endl;
  return (os);
}
// Other Members
int Box::getDim() const
{
  return this->dim;
}
bool Box::isEmpty() const
{
  if (this->dim==0 || this->data==NULL) return true;
  for (int k=0; k<this->dim; k++){
    if (this->data[k].isEmpty()) return true;
  }
  return false;
}
int Box::mainAxis() const
{
  double widthmax=this->operator [](1).width();
  int kmax=1;
  for (int k=2; k<=this->dim; k++){
    if (this->operator [](k).width()>widthmax){
      kmax=k;
      widthmax=this->operator [](k).width();
    };
  }
  return kmax;
}
double   Box::volume() const
{
  if (this->isEmpty()) return 0.0;
  double vol=1;
  for (int i=1;i<=this->dim;i++)
    vol=vol*this->operator [](i).width();
  return vol;
}
double Box::width() const
{
  if (this->isEmpty()) return 0.0;
  return this->operator [](this->mainAxis()).width();
}
Iboolean Box::isIn(Box &b)
{
  if (this->isDisjoint(b)) return Iboolean(ifalse);
  if (this->isSubset(b)) return Iboolean(itrue);
  return Iboolean(iperhaps);
}
Iboolean Box::isInInterior(Box &b)
{
  if (this->isDisjoint(b)) return Iboolean(ifalse);
  if (this->isSubsetInterior(b)) return Iboolean(itrue);
  return Iboolean(iperhaps);
}
bool Box::isJoint(Box &b)
{
  for (int i=1;i<=this->dim;i++){
    if ((this->operator [](i)).areJoint(b[i]))
      return true;
  }
  return false;
}
bool Box::isDisjoint(Box &b)
{
  for (int i=1;i<=this->dim;i++){
    if ((this->operator [](i)).isDisjoint(b[i]))
      return true;
  }
  return false;
}
bool Box::isSubset(Box &b)
{
  for (int i=1;i<=this->dim;i++){
    if ((this->operator [](i)).isSubset(b[i]) == false)
      return false;
  }
  return true;
}
bool Box::isSubsetInterior(Box &b)
{
  for (int i=1;i<=this->dim;i++){
    if ((this->operator [](i)).isSubsetInterior(b[i]) == false)
      return false;
  }
  return true;
}
// Friends Functions
void Bisect(Box& x, Box& x1, Box& x2)
{
  BisectAlong(x,x1,x2,x.mainAxis());
}

void BisectInteger(Box& x, Box& x1, Box& x2)
{
  x1 = x;
  x2 = x;
  int mAxis = x.mainAxis();
  double mx = x[mAxis].centerInt();
  x1[mAxis].sup = mx;
  x2[mAxis].inf = mx;
}
void BisectAlong(Box& x, Box& x1, Box& x2, int i)
{
  x1 = x;
  x2 = x;
  double mx = x[i].center();
  x1[i].sup = mx;
  x2[i].inf = mx;
}
Box Inter(const Box& X, const Box& Y)
{
  Box Ans(X.getDim());
  if ((X.isEmpty())||(Y.isEmpty())) {return Ans;}
  for (int k=1; k<=X.getDim(); k++){
    Ans[k] = inter(X[k],Y[k]);
    if (Ans[k].isEmpty()) {Update(Ans); return Ans;}
  }
  return Ans;
}
Box hull(const Box &a, const Box &b)
{
    if (a.isEmpty()) return b;
    if (b.isEmpty()) return a;
    Box r(a.getDim());
    for (int k=1; k<=a.getDim(); k++)
      r[k] = hull(a[k],b[k]);
    return r;
}
void Update(Box& X)
{
  for (int i=1; i<=X.getDim(); i++){
    if (X[i].isEmpty()){
      for (int j=1; j<=X.getDim(); j++)
        X[j]=Interval(); return;
    }
  }
}
list<Box> qBissect(Box &pOut,Box &pIn)
{
  list<Box> Lb;
  //This function works only for dimension 2 boxes
  if (pOut.getDim() != 2 || pIn.getDim() != 2)
    return Lb;
  Box b(Interval(pOut[1].inf,pIn[1].inf),pOut[2]);
  if (!b[1].isDegenerated() && !b[2].isDegenerated())
    Lb.push_back(b);
  b = Box(Interval(pIn[1].inf,pIn[1].sup),Interval(pOut[2].inf,pIn[2].inf));
  if (!b[1].isDegenerated() && !b[2].isDegenerated())
    Lb.push_back(b);
  b = Box(Interval(pIn[1].inf,pIn[1].sup),Interval(pIn[2].sup,pOut[2].sup));
  if (!b[1].isDegenerated() && !b[2].isDegenerated())
    Lb.push_back(b);
  b = Box(Interval(pIn[1].sup,pOut[1].sup),pOut[2]);
  if (!b[1].isDegenerated() && !b[2].isDegenerated())
    Lb.push_back(b);
  return Lb;
}

Box  inflate  (const Box &X,double eps)
{
    Box ret(X.getDim());
    for (int i=1; i<=X.getDim(); i++){
        ret[i]=inflate(X[i],eps);
    }
    return ret;
}


