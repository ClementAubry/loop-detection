#ifndef __BOX__
#define __BOX__

#include "interval.h"
#include "iboolean.h"
#include <iostream>
#include <string>
#include <list>

using std::list;

class Box
{
private:
    // Interval vector data.
    Interval* data;
    // dimension of Interval vector data
    int dim;

public:
    // Constructor
    Box ();
    Box (int);
    Box (Interval, Interval);
    Box (Interval, Interval, Interval);
    Box (const Box&);
    // Destructor.
    ~Box ();

    // Members Overloaded Operators
    Interval& operator[] (int)  const;
    Box& operator=(const Box &);
    // Other Overloaded Operators
    friend std::ostream& operator<<(std::ostream&, const Box&);
    friend bool operator==(const Box&,const Box&);
    friend Box	operator+(const Box&,const Box&);
    friend Box	operator-(const Box&);
    friend Box	operator-(const Box&,const Box&);
    friend Box	operator*(const Interval&,const Box&);
    friend Box	operator*(const Box&,const Interval&);

    friend std::ostream & operator << (std::ostream & out, Box & T);

    // Other Members
    int      getDim() const;
    bool     isEmpty() const;
    int      mainAxis() const;
    double   volume () const;
    double   width() const;
    Iboolean isIn(Box &);
    Iboolean isInInterior(Box &);
    bool     isJoint(Box &);
    bool     isDisjoint(Box &);
    bool     isSubset(Box &);
    bool     isSubsetInterior(Box &);
    // Friends Functions
    friend void Bisect (Box&, Box&, Box&);
    friend void BisectInteger (Box&, Box&, Box&);
    friend void BisectAlong (Box&, Box&, Box&, int);
    friend Box  Inter(const Box& X, const Box& Y);
    friend Box  hull(const Box& X, const Box& Y);
    friend void Update(Box&);

    friend list<Box> qBissect(Box &Pout,Box &Pin);
    friend Box  inflate  (const Box &X,double eps);

};

#endif
