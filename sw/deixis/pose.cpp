// preprocessor directives
#include "pose.h"
using namespace std;



// <public constructors>

//
Pose::Pose(const PositionVector pv, const double deg)
   : PositionVector(pv), heading(deg)
{
}  // Pose(const PositionVector &, const double)



//
Pose::Pose(const PositionVector pv, const HeadingVector hv)
   : PositionVector(pv), heading(hv.angle())
{
}  // Pose(const PositionVector &, const HeadingVector &)



//
Pose::Pose(const Pose &pose): PositionVector(pose), heading(pose.heading)
{
}  // Pose(const Pose &)



// <public destructors>

//
Pose::~Pose()
{
}  // ~Pose()



// <public mutator functions>

//
bool Pose::setPosition(const double dx, const double dy)
{
   return set(dx, dy);
}  // setPosition(const double, const double)



//
bool Pose::setPosition(const double dx, const double dy, const double dz)
{
   return set(dx, dy, dz);
}  // setPosition(const double, const double, const double)



//
bool Pose::setPosition(const PositionVector &pv)
{
   return set(pv);
}  // setPosition(const PositionVector &)



//
bool Pose::setHeading(const double deg)
{
   heading = scaleDegrees(deg);
   return true;
}  // setHeading(const double)



//
bool Pose::setHeading(const HeadingVector &hv)
{
   return setHeading(hv.angle());
}  // setHeading(const HeadingVector &)



//
bool Pose::rotateHeading(const double deg)
{
   heading = scaleDegrees(heading + deg);
   return true;
}  // rotateHeading(const double)



// <public accessor functions>

//
PositionVector Pose::getPosition() const
{
   return *this;
}  // getPosition()



//
double Pose::getHeading() const
{
   return heading;
}  // getHeading()



//
HeadingVector Pose::getHeadingVector() const
{
   HeadingVector hv;
   hv.setPolar(1.0, heading);
   return hv;
}  // getHeadingVector()



// <public utility functions>

//
PositionVector Pose::getRelationshipTo(PositionVector pv)
{
   return PositionVector(pv - *this).rotated(-getHeading());
}  // getRelationshipTo(PositionVector)



//
void Pose::print()
{
   print(cout);
}  // print()



//
void Pose::print(ostream &out)
{
   out << *this << endl;
}  // print(ostream &)



// <public friend functions>

//
ostream& operator <<(ostream &out, const Pose &pose)
{
   out << PositionVector(pose)
       << " | [" << pose.heading << CHAR_DEGREE_SIGN << "]";
   return out;
}  // <<(ostream &, const Pose &)
