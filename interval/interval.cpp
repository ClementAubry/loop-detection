#include "interval.h"

#if defined(_MSC_VER) | defined(__BORLANDC__)
// Enable the use of isnan().
#include <float.h>
#define isnan _isnan
// Used to define NAN (Not A Number).
const unsigned long nan[2]={0xffffffff, 0xffffffff};
const double nan_double = -*(double*)nan;
#endif // defined(_MSC_VER) | defined(__BORLANDC__)
#ifdef _MSC_VER
// Enable additional features in math.h, which is included by cmath.
#define _USE_MATH_DEFINES
#endif // _MSC_VER

#include <cmath>

using namespace std;

const double PI_2 = M_PI_2;
const double TROIS_PI_2 = 3*PI_2;
const double CINQ_PI_2 = 5*PI_2;
const double SEPT_PI_2 = 7*PI_2;
const double DEUX_PI = 4*PI_2;
const double PI = 2*PI_2;

// Constructors
Interval::Interval():inf(NAN),sup(NAN)
{}
Interval::Interval (const double& m):inf(m),sup(m)
{}
Interval::Interval (const double& a, const double& b):inf(a),sup(b)
{}
Interval::Interval (const Interval& a):inf(a.inf),sup(a.sup)
{}
// Desctuctor
Interval::~Interval()
{}
// MEMBERS, OVERLOAD (unary operators)
void Interval::operator= (const Interval &b)
{
    this->inf = b.inf;
    this->sup = b.sup;
}
Interval operator+ (const Interval& x,const Interval& y)
{
    if (x.isEmpty()) return x;
    if (y.isEmpty()) return y;
    return Interval(x.inf+y.inf,x.sup+y.sup);
}
Interval operator+ (const Interval& x,const double& y)
{
    Interval z(x.inf+y, x.sup+y);
    return  z;
}
Interval operator+ (const double& y, const Interval& x)
{
    Interval z(x.inf+y, x.sup+y);
    return  z;
}
Interval operator- (const Interval& a)
{
    if (a.isEmpty()) return a;
    else  return(Interval(-(a.sup),-(a.inf)));
}
Interval operator- (const Interval& x, const Interval& y)
{   return(x+(-y));}

Interval operator- (const Interval& x, const double& y)
{   return(x+(-y));}

Interval operator- (const double& x, const Interval& y)
{   return(x+(-y));}

Interval operator* (const Interval& x, const Interval& y)
{
    Interval r;
    if (x.isEmpty()) return x;
    if (y.isEmpty()) return y;
    double x1=x.inf,y1=y.inf,x2=x.sup,y2=y.sup;
    if (x.isDegenerated() && y.isDegenerated()) return(Interval(x1*y1));
    if (x1>=0)      {
        if  (y1>=0) return Interval(x1*y1,x2*y2);
        else if  (y2<=0) return Interval(x2*y1,x1*y2);
        else return Interval(x2*y1,x2*y2);
    }
    else if (x2<=0) {
        if  (y1>=0) return Interval(x1*y2,x2*y1);
        else if (y2<=0) return Interval(x2*y2,x1*y1);
        else return Interval(x1*y2,x1*y1);
    }
    else {
        if (y1>=0) return Interval(x1*y2,x2*y2);
        else if (y2<=0) return Interval(x2*y1,x1*y1);
        else return Interval(min(x1*y2,x2*y1),max(x1*y1,x2*y2));
    }
}
Interval operator* (const Interval& x, const double& y)
{
    Interval r;
    if (x.isEmpty()) return x;
    double x1=x.inf,x2=x.sup;
    if (x.isDegenerated()) return(Interval(x1*y));
    return Interval(min(x1*y,x2*y),max(x1*y,x2*y));
}
Interval operator* (const double& y, const Interval& x)
{
    Interval r;
    if (x.isEmpty()) return x;
    double x1=x.inf,x2=x.sup;
    if (x.isDegenerated()) return(Interval(x1*y));
    return Interval(min(x1*y,x2*y),max(x1*y,x2*y));
}
Interval operator/  (const Interval& a, const Interval& b)
{
    Interval r;
    if (a.isEmpty()) return a;
    if (b.isEmpty()) return b;
    if (b.isDegenerated())
    {
        double b1=b.inf;
        if (b1==0) return(Interval(-oo,oo));
        else return (1/b1)*a;
    }
    if (b.inf > 0)
    {
        if      (a.inf>=0) return Interval(a.inf/b.sup,a.sup/b.inf);
        else if (a.sup<0)  return Interval(a.inf/b.inf,a.sup/b.sup);
        else                return Interval(a.inf/b.inf,a.sup/b.inf);
    }
    else if (b.sup<0)
    {
        if      (a.inf>=0) return Interval(a.sup/b.sup,a.inf/b.inf);
        else if (a.sup<0)  return Interval(a.sup/b.inf,a.inf/b.sup);
        else                return Interval(a.sup/b.sup,a.inf/b.sup);
    }
    else                    return Interval(-oo,oo);
}
Interval operator/  (const Interval& a, const double& b)
{
    if (b==0)
        return(Interval(-oo,oo));
    else
        return (1/b)*a;
}
Interval operator& (const Interval& a,const Interval& b)
{
    if (a.isEmpty() || b.isEmpty()) return Interval();
    Interval r(0,0);
    if ((a.inf>b.sup)||(b.inf>a.sup)) return Interval();
    if (a.inf<=b.inf) r.inf=b.inf; else r.inf=a.inf;
    if (a.sup>=b.sup) r.sup=b.sup; else r.sup=a.sup;
    return Interval(r);
}
Interval operator| (const Interval& a,const Interval& b)
{
    if (a.isEmpty()) return b;
    if (b.isEmpty()) return a;
    Interval r;
    double bsup=b.sup;
    double binf=b.inf;
    double asup=a.sup;
    double ainf=a.inf;
    r.inf = (ainf < binf)? ainf : binf;
    r.sup = (asup > bsup)? asup : bsup;
    return Interval(r);
}
bool operator== (const Interval &a,const Interval &b)
{
    if (a.isEmpty() && b.isEmpty())
        return true;
    if ((a.inf==b.inf) && (a.sup==b.sup))
        return true;
    else
        return false;
}
bool operator< (const Interval &a,const Interval &b)
{
    return (a.inf>b.inf)&&(a.sup<b.sup);
}
bool operator<= (const Interval &a,const Interval &b)
{
    return (a.inf>=b.inf)&&(a.sup<=b.sup);
}
bool operator!= (const Interval &a,const Interval &b)
{
    if (a.isEmpty() && b.isEmpty()) return false;
    if (a.isEmpty() || b.isEmpty()) return true;
    return (a.sup<b.inf)||(b.sup<a.inf);
}
ostream& operator<< (ostream& os, const Interval& a)
{
    if (a.isEmpty())
        os<<"Empty Interval";
    else
        os << "[" << a.inf << ", " << a.sup << "] ";
    return os;
}
//OTHER MEMBERS

double Interval::center() const
{
    if (!this->isEmpty()) return (this->sup+this->inf)/2;
    return this->inf;
}
int Interval::centerInt() const
{
    if (!this->isEmpty()) return (int)((this->sup+this->inf)/2);
    return this->inf;
}
bool Interval::contains(double a)  const
{
    if (this->isEmpty()) return false;
    return (this->inf<=a && a<=this->sup);
}
bool Interval::isDegenerated()  const
{
    if (this->isEmpty())         return true;
    if (this->inf == this->sup)  return true;
    else return false;
}

Iboolean Interval::isIn(Interval &b)
{
    if (this->isDisjoint(b)) return Iboolean(ifalse);
    if (this->isSubset(b)) return Iboolean(itrue);
    return Iboolean(iperhaps);
}
Iboolean Interval::isInInterior(Interval &b)
{
    if (this->isDisjoint(b)) return Iboolean(ifalse);
    if (this->isSubsetInterior(b)) return Iboolean(itrue);
    return Iboolean(iperhaps);
}

bool Interval::areJoint(Interval &b)
{
    if (this->isEmpty() || b.isEmpty()) return true;
    return (this->sup==b.inf)||(b.sup==this->inf);
}
bool Interval::isDisjoint(Interval &b)
{
    if (this->isEmpty() || b.isEmpty()) return false;
    return (this->sup<b.inf)||(b.sup<this->inf);
}

bool Interval::isSubset(Interval &b)
{
  return (b.inf<=this->inf)&&(this->sup<=b.sup);
}
bool Interval::isSubsetInterior(Interval &b)
{
  return (b.inf<this->inf)&&(this->sup<b.sup);
}



bool Interval::isRealNegative() const
{
  return ((this->inf == this->sup) && (this->inf < 0));
}


bool Interval::isEmpty() const
{
//    if (isnan(this->inf) || isnan(this->sup) || (this->inf>this->sup))
//        return true;
//    else
//        return false;
  return (isnan(this->inf) || isnan(this->sup) || (this->inf>this->sup));
}
double Interval::width () const
{
    return (this->sup-this->inf);
}

// OTHER FRIENDS
Interval inter(const Interval &a, const Interval &b)
{
    if (a.isEmpty() || b.isEmpty()) return Interval();
    Interval r(0,0);
    if ((a.inf>b.sup)||(b.inf>a.sup)) return Interval();
    if (a.inf<=b.inf) r.inf=b.inf; else r.inf=a.inf;
    if (a.sup>=b.sup) r.sup=b.sup; else r.sup=a.sup;
    return Interval(r);
}
Interval hull(const Interval &a, const Interval &b)
{
    if (a.isEmpty()) return b;
    if (b.isEmpty()) return a;
    Interval r(0,0);
    if (a.inf<=b.inf) r.inf=a.inf; else r.inf=b.inf;
    if (a.sup>=b.sup) r.sup=a.sup; else r.sup=b.sup;
    return r;
}



// MATH
Interval abs  (const Interval &a)
{
    if (a.isDegenerated()) return fabs(a.inf);
    Interval r(fabs(a.inf),fabs(a.sup));
    if (a.contains(0))
        r.inf = 0;
    return r;
}
double absMax(const Interval &x)
{
    double a = fabs(x.inf), b = fabs(x.sup);
    return fmax(a,b);
}
Interval  cos(const Interval &a)
{
    return sin(a+PI_2);
}
Interval  exp(const Interval &a)
{
    if (a.isDegenerated()) return Interval(exp(a.inf),exp(a.inf)) ;
    else return Interval(exp(a.inf),exp(a.sup));
}
Interval  inflateAdd    (const Interval &a,double eps)
{
    Interval r;
    if (a.isEmpty())
    {
        r.inf=a.inf-eps;
        r.sup=a.inf+eps;
    }
    else
    {
        r.inf=a.inf-eps;
        r.sup=a.sup+eps;
    }
    return Interval(r);
}
Interval  inflate    (const Interval &a,double eps)
{
    Interval r;
    double w = a.width();
    r.inf=a.center()-w*eps;
    r.sup=a.center()+w*eps;
    return Interval(r);
}
Interval  log        (const Interval &a)
{
    if (a.contains(0)) return Interval(-oo,log(a.sup));
    if (a.sup<0) return Interval(-oo,-oo);
    Interval b=abs(a);
    return Interval(log(b.inf),log(b.sup));
}
Interval max (const Interval& x,const Interval& y)
{
    return (-min(-x,-y));
}
Interval max (const Interval& x, const Interval& y,const Interval& z)
{
    return(max(max(x,y),z));
}

double max (vector<double>& x)
{
    double d=-oo;
    for (unsigned int i=0;i<x.size();i++)
        d=max(x[i],d);
    return d;
}

Interval min (const Interval& x, const Interval& y,const Interval& z)
{
    return(min(min(x,y),z));
}

Interval min (const Interval& x, const Interval& y)
{
    if (x.isEmpty()||y.isEmpty()) return Interval();
    double a = min(x.sup,y.sup);
    double b = min(x.inf,y.inf);
    if ((x.sup==x.inf)&&(y.sup==y.inf)) return Interval(b);
    return Interval(a,b);
}
double min (vector<double>& x)
{
    double d=oo;
    for (unsigned int i=0;i<x.size();i++)
        d=min(x[i],d);
    return d;
}
Interval modulo(const Interval& a, double x)
{
    if ((a.inf>=0)&&(a.inf<x)) return (a);
    int k = (long)floorl((a.inf/x));
    double offset = x * k;
    return Interval(a.inf-offset,a.sup-offset);
}
Interval  notIn    ( Interval &a, Interval &b)
{
    if (a!=b) return (a);
    if (b<=a) return (a);
    if (a.inf < b.inf) return Interval(a.inf,b.inf);
    else
        return Interval(b.sup,a.sup);
}
double power (double x, int n, int RndMode )
{
    int  ChangeRndMode;     // for x < 0 and odd n
    double p, z;
    ChangeRndMode = ( (x < 0.0) && (n % 2 == 1) );
    if (ChangeRndMode) {z=-x; RndMode=-RndMode;}  else     z= x;
    p = 1.0;
    switch (RndMode) {                             // Separate while-loops used
    case -1 :   while (n > 0)
        {if (n % 2 == 1) p = p*z;   //--------------------------
            n = n / 2; if (n > 0) z = z*z;}break;
                        case +1 :   while (n > 0)
        {if (n % 2 == 1) p = p*z;
                                n = n / 2;
                                if (n > 0) z = z*z;}break;  }
    if (ChangeRndMode)   return -p; else  return p;
}


Interval power(const Interval &x,int n)
{
    int  m;
    double Lower, Upper;
    if (n == 0) return(Interval(1.0,1.0));
    if (x.isDegenerated())  return Interval(x.inf);
    if (n > 0)  m = n;
    else  m = -n;
    if ( (0.0 < x.inf) || (m % 2 == 1) )
    {
        Lower = power(x.inf,m,-1);
        Upper = power(x.sup,m,+1);
    }
    else if (0.0 > x.sup)
    {
        Lower = power(x.sup,m,-1);
        Upper = power(x.inf,m,+1);
    }
    else
    {
        Lower = 0.0;
        Upper = power(absMax(x),m,+1);
    }
    if (n > 0) return(Interval(Lower,Upper));
    else   return(1.0/Interval(Lower,Upper));    // if 0 in 'x'.
}
Interval  sin(const Interval &a)
{
    if (a.isEmpty()) return Interval();
    if (a.isDegenerated()) return sin(a.inf);
    Interval b;
    double sin1,sin2,r1,r2;
    b = modulo(a,DEUX_PI);
    if (a.width()>DEUX_PI) return (Interval(-1,1));
    sin1=sin(b.inf);
    sin2=sin(b.sup);
    if ((b.inf < TROIS_PI_2)&&(b.sup > TROIS_PI_2)) r1=-1.0;
    else if ((b.inf < SEPT_PI_2)&&(b.sup > SEPT_PI_2)) r1=-1.0;
    else r1=((sin1 < sin2)? sin1 : sin2);
    if ((b.inf < PI_2)&&(b.sup > PI_2)) r2=1.0;
    else if ((b.inf < CINQ_PI_2)&&(b.sup > CINQ_PI_2)) r2=1.0;
    else r2=((sin1 > sin2)? sin1 : sin2);
    return inter(Interval(-1,1),Interval(r1,r2));

}

Interval sqr(const Interval &a)
{
    double a1=a.inf,a2=a.sup;
    if (a.isDegenerated())  return Interval(a1*a1);
    if (a1>=0)   return Interval(a1*a1,a2*a2);
    if (a2<=0)   return Interval(a2*a2,a1*a1);
    if (fabs(a1)>fabs(a2))  return Interval(0,a1*a1);
    else                    return Interval(0,a2*a2);
}

Interval sqrt(const Interval &a)
{
    double a1=a.inf,a2=a.sup;
    Interval r;
    if (a2<0)  { return r;};
    if (a1<0)  {a1=0;};
    r=Interval(sqrt(a1),sqrt(a2));
    return r;
}
Interval step(const Interval& X)
{ //if (X.isEmpty) return(interval());
    if (X.inf>0) return (Interval(1));
    if (X.sup<0) return(Interval(0));
    return (Interval(0,1));
}
Interval  tan        (const Interval &a)
{
    if (a.isDegenerated()) return tan(a.inf);
    if (a.width()>PI) return (Interval(-oo,oo));
    Interval b = modulo(a,PI);
    if (PI_2<=b.inf) {b=b-PI;}
    if (b.contains(PI_2)) return (Interval(-oo,oo));
    return Interval(tan(b.inf),tan(b.sup));
}

void Cplus(Interval& Z, Interval& Y, Interval& X, int sens)
        /* Z=Y+X         =>  sens=1;
   Y=Z-X; X=Z-Y  =>  sens=-1; */
{ if (sens==1)  {Z=inter(Z,Y+X);}
    if (sens==-1) {Y=inter(Y,Z-X); X=inter(X,Z-Y);}}
//----------------------------------------------------------------------
void Cplus(double& Z, Interval& Y, Interval& X, int sens)
{ //Luc
    if (sens==-1) {Y=inter(Y,Z-X); X=inter(X,Z-Y);}}
//----------------------------------------------------------------------
void Cmoins(Interval& Z, Interval& Y, Interval& X, int sens)
        /* Z=Y-X           =>  sens=1;
   Y=Z+X; X=Y-Z    =>  sens=-1; */
{ if (sens==1)  {Z=inter(Z,Y-X);}
    if (sens==-1) {Y=inter(Y,Z+X); X=inter(X,Y-Z);}}
//----------------------------------------------------------------------
void Cmoins(double& Z, Interval& Y, Interval& X, int sens)
        // Luc
{ if (sens==-1) {Y=inter(Y,Z+X); X=inter(X,Y-Z);}}
//----------------------------------------------------------------------
void Cmoins(Interval& Z, Interval& Y, double& x, int sens)
{
    if (sens==1)  {Z=inter(Z,Y-x);}
    if (sens==-1) {Y=inter(Y,Z+x);}
}
void Cprod(Interval& Z, Interval& Y, Interval& X, int sens)
        /*  Z=Y*X           =>  sens=1;
   Y=Z/X; X=Z/Y    =>  sens=-1;  */
{
    if (sens==1)  {Z=inter(Z,Y*X);}
    if (sens==-1) {
        //modifs ???
        if (Z.contains(0) == false){
            Interval Xd, Xg;
            Interval Yd, Yg;
            Yg=inter(Y,Interval(-oo,0)), Yd=inter(Y,Interval(0,oo));
            Xg=inter(X,Interval(-oo,0)), Xd=inter(X,Interval(0,oo));

            if (Z.inf>0){
                if (  Xd.sup*Yd.sup<Z.inf){
                    Xd=Interval();
                    Yd=Interval();
                }else if ((Xd.sup!=0)&&(Yd.sup!=0)){
                    Xd=inter(Xd,Interval(Z.inf/Yd.sup,oo));
                    Yd=inter(Yd,Interval(Z.inf/Xd.sup,oo));
                }
                if (  Xg.inf*Yg.inf<Z.inf){
                    Xg=Interval();
                    Yg=Interval();
                }else if ((Xg.inf!=0)&&(Yg.inf!=0)){
                    Xg=inter(Xg,Interval(Z.inf/Yg.inf,-oo));
                    Yg=inter(Yg,Interval(Z.inf/Xg.inf,-oo));
                }
                X=inter(X,hull(Xd,Xg));
                Y=inter(Y,hull(Yd,Yg));
            }
            if (Z.sup<0){
                if (  Xg.inf*Yd.sup>Z.sup){
                    Xg=Interval();
                    Yd=Interval();
                }else if ((Xg.inf!=0)&&(Yd.sup!=0))
                {
                    Xg=inter(Xg,Interval(Z.sup/Yd.sup,-oo));
                    Yd=inter(Yd,Interval(Z.sup/Xg.inf,oo));
                }
                if (  Xd.sup*Yg.inf>Z.sup) {
                    Xd=Interval();
                    Yg=Interval();
                }else if ((Xd.sup!=0)&&(Yg.inf!=0))
                {
                    Xd=inter(Xd,Interval(Z.sup/Yg.inf,oo));
                    Yg=inter(Yg,Interval(Z.sup/Xd.sup,-oo));
                }
                X=inter(X,hull(Xd,Xg));
                Y=inter(Y,hull(Yd,Yg));
            }
        }
        Y=inter(Y,Z/X); X=inter(X,Z/Y); //Y=Inter(Y,Z/X);
    }
}
