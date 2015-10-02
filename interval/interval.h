#ifndef INTERVAL_H
#define INTERVAL_H

#include <cmath>
#ifndef INFINITY
#define INFINITY HUGE_VAL
#endif // INFINITY
// infinity is denoted by oo
#define oo INFINITY
#if defined(_MSC_VER) | defined(__BORLANDC__)
// Used to define NAN (Not A Number).
#ifndef NAN
extern const unsigned long nan[2];
extern const double nan_double;
#define NAN nan_double
#endif // NAN
#endif // defined(_MSC_VER) | defined(__BORLANDC__)

#include <iostream>
#include <vector>
#include "iboolean.h"
#include <string>

using std::string;

class Interval
{
public:
    // ACCESSIBLES MEMBERS
    double inf; // Lower bound.
    double sup; // Upper bound.

    // Constructors
    Interval();
    Interval(const double&);
    Interval(const double&, const double&);
    Interval(const Interval&);

    // Destructor.
    ~Interval();

    // MEMBERS, OVERLOAD (unary operator)
    void operator=( const Interval&);

    // FRIENDS, OVERLOAD (binary operators)
    friend Interval operator+(const Interval&,const Interval&);
    friend Interval operator+(const Interval&,const double&);
    friend Interval operator+(const double&, const Interval&);

    friend Interval operator-(const Interval&);
    friend Interval operator-(const Interval&, const Interval&);
    friend Interval operator-(const Interval&, const double&);
    friend Interval operator-(const double&, const Interval&);

    friend Interval operator*(const Interval&, const Interval&);
    friend Interval operator*(const Interval& x, const double& y);
    friend Interval operator*(const double& y, const Interval& x);

    friend Interval operator/(const Interval& ,const Interval&);
    friend Interval operator/(const Interval& ,const double& y);

    friend Interval operator& (const Interval&,const Interval&);
    friend Interval operator| (const Interval&,const Interval&);
    friend bool     operator==(const Interval&,const Interval&);
    friend bool     operator< (const Interval&,const Interval&);
    friend bool     operator<=(const Interval&,const Interval&);
    friend bool     operator!=(const Interval&,const Interval&);

    // Output format
    friend std::ostream& operator<< (std::ostream&, const Interval&);

    // OTHER MEMBERS
    bool        isRealNegative() const;
    bool        isEmpty()        const;
    bool        isDegenerated()  const;
    bool        contains(double) const;
    double      width()          const;
    double      center()         const;
    int         centerInt()      const; //Use with caution, return a int truncature of the center
    Iboolean    isIn(Interval &b);
    Iboolean    isInInterior(Interval &b);
    bool        areJoint(Interval &b);
    bool        isDisjoint(Interval &b);
    bool        isSubset(Interval &b);
    bool        isSubsetInterior(Interval &b);

    // OTHER FRIENDS
    friend Interval inter   (const Interval&, const Interval&);
    friend Interval hull    (const Interval&, const Interval&);

    // Fonctions MATHEMATIQUES
    friend Interval abs        (const Interval&);
    friend double   absMax     (const Interval&);              // Absolute maximum of
    friend Interval cos        (const Interval&);
    friend Interval exp        (const Interval&);
    friend Interval inflateAdd    (const Interval&,double);
    friend Interval inflate    (const Interval&,double);
    friend Interval notIn      ( Interval&, Interval&);
    friend Interval log        (const Interval&);
    friend Interval max        (const Interval&,const Interval&);
    friend Interval max        (const Interval&,const Interval&,const Interval&);
    friend double   max        (std::vector<double>& x);
    friend Interval min        (const Interval&,const Interval&);
    friend Interval min        (const Interval&,const Interval&,const Interval&);
    friend double   min        (std::vector<double>& x);
    friend Interval modulo     (const Interval&, double);
    friend double   power      (double, int, int);
    friend Interval power      (const Interval&,int);
    friend Interval sin        (const Interval&);
    friend Interval sqr        (const Interval&);
    friend Interval sqrt       (const Interval&);
    friend Interval step       (const Interval& X);
    friend Interval tan        (const Interval&);

    friend void Cplus (Interval&, Interval&, Interval&, int);
    friend void Cplus (double&,   Interval&, Interval&, int);
    friend void Cmoins(Interval&, Interval&, Interval&, int);
    friend void Cmoins(double&,   Interval&, Interval&, int);
    friend void Cmoins(Interval&, Interval&, double& x, int);
    friend void Cprod (Interval&, Interval&, Interval&, int);

};
#endif // INTERVAL_H
