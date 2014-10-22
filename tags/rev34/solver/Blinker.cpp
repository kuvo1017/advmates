#include "Blinker.h"

//======================================================================
Blinker::Blinker()
{
    state = 0;
}
//======================================================================
Blinker::~Blinker()
{
}
//======================================================================
bool Blinker::isNone() const
{
    return (state == 0);
}
//======================================================================
bool Blinker::isLeft() const
{
    return (state == 1);
}
//======================================================================
bool Blinker::isRight() const
{
    return (state == 2);
}
//======================================================================
bool Blinker::isHazard() const
{
    return (state == 3);
}
//======================================================================
int Blinker::getState() const
{
    return state;
}
//======================================================================
RelativeDirection Blinker::turning() const
{
    switch (state)
    {
    case 0:
        return RD_STRAIGHT;
    case 1:
        return RD_LEFT;
    case 2:
        return RD_RIGHT;
    case 3:
    default:
        return RD_NONE;
    }
}

//======================================================================
bool Blinker::operator==(Blinker& another) const
{
    return (state == another.state);
}
//======================================================================
void Blinker::setNone()
{
    state = 0;
}
void Blinker::setLeft()
{
    state = 1;
}
void Blinker::setRight()
{
    state = 2;
}
void Blinker::setHazard()
{
    state = 3;
}
//======================================================================
