#include "iboolean.h"
using namespace std;


Iboolean::Iboolean()
{
    this->value=iempty;
}

Iboolean::Iboolean (bool b)
{
    if (b) this->value=itrue;
    else this->value=ifalse;
}

Iboolean::Iboolean (IBOOLEAN tt)
{
    this->value = tt;
}

Iboolean::Iboolean (const Iboolean& t)
{
    *this = t;
}

Iboolean::~Iboolean()
{
}

Iboolean operator&& (Iboolean x, Iboolean y)
{
    if ( (x.value==ifalse) || (y.value==ifalse) ) return Iboolean(ifalse);
    if ( (x.value==itrue) && (y.value==itrue) ) return Iboolean(itrue);
    return Iboolean(iperhaps);
}

Iboolean operator|| (Iboolean& x, Iboolean& y)
{
    if ( (x.value==itrue) || (y.value==itrue) ) return Iboolean(itrue);
    if ( (x.value==iperhaps) || (y.value==iperhaps) ) return Iboolean(iperhaps);
    return Iboolean(ifalse);
}
Iboolean operator|| (Iboolean x, Iboolean y)
{
    if ( (x.value==itrue) || (y.value==itrue) ) return Iboolean(itrue);
    if ( (x.value==iperhaps) || (y.value==iperhaps) ) return Iboolean(iperhaps);
    return Iboolean(ifalse);
}

bool operator== (Iboolean x, Iboolean y)
{	return (x.value==y.value);}

bool operator!= (Iboolean x, Iboolean y)
{	return (x.value!=y.value);}


Iboolean operator! (Iboolean& x)
{
    if (x.value == itrue) return Iboolean(ifalse);
    if (x.value == ifalse) return Iboolean(itrue);
    return Iboolean(iperhaps);
}
Iboolean operator! (Iboolean x)
{
    if (x.value == itrue) return Iboolean(ifalse);
    if (x.value == ifalse) return Iboolean(itrue);
    return Iboolean(iperhaps);
}

ostream& operator<<(ostream& os, const Iboolean& v)
{
    if (v.value==itrue)
        os<<"ITrue \t";
    if (v.value==ifalse)
        os<<"IFalse \t";
    if (v.value==iperhaps)
        os<<"Iperhaps \t";
    if (v.value==iempty)
        os<<"Iempty \t";
    return (os);
}


Iboolean Inter(Iboolean a, Iboolean b)
{
    if ((a.value == itrue)&&(b.value == ifalse)) return Iboolean(iempty);
    if ((a.value == ifalse)&&(b.value == itrue)) return Iboolean(iempty);
    if ((a.value == itrue)||(b.value == itrue)) return Iboolean(itrue);
    if ((a.value == ifalse)||(b.value == ifalse)) return Iboolean(ifalse);
    return Iboolean(iperhaps);
}

std::string Iboolean::toString(void)
{
  std::string s;
  if (this->value==itrue)
      s = "ITrue";
  if (this->value==ifalse)
      s = "IFalse";
  if (this->value==iperhaps)
      s = "Iperhaps";
  if (this->value==iempty)
      s = "Iempty";
  return (s);
}
