#ifndef pistonClass_hpp
#define pistonClass_hpp
#include "vex.h"

class Piston
{
private:
    vex::digital_out piston;
    bool state = false;

public:
    Piston(vex::digital_out pistonPort) : piston(pistonPort) {}
    void toggle()
    {
        state = !state;
        piston.set(state);
    }
    void set(bool state)
    {
        this->state = state;
        piston.set(state);
    }
    bool get()
    {
        return state;
    }
};

#endif // pistonClass_hpp