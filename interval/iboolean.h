#ifndef IBOOLEAN_H
#define IBOOLEAN_H
#include <iostream>
enum IBOOLEAN {itrue, ifalse, iperhaps, iempty};

class Iboolean{
public:
    // The Iboolean value
    IBOOLEAN value;
public:
    // Constructor
    Iboolean();
    Iboolean (bool);
    Iboolean (IBOOLEAN);
    Iboolean (const Iboolean&);
    // Destructor.
    ~Iboolean();

    std::string toString(void);

    friend Iboolean operator&& (Iboolean, Iboolean);
    friend Iboolean operator|| (Iboolean&, Iboolean&);
    friend Iboolean operator|| (Iboolean, Iboolean);
    friend Iboolean operator!  (Iboolean&);
    friend Iboolean operator!  (Iboolean);
    friend bool     operator== (Iboolean, Iboolean);
    friend bool	    operator!= (Iboolean, Iboolean);
    friend std::ostream& operator<< (std::ostream& os, const Iboolean&);
    friend Iboolean Inter(Iboolean a, Iboolean b);
};

#endif // IBOOLEAN_H
