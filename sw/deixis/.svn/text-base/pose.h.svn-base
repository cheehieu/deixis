// preprocessor directives
#ifndef POSE_H
#define POSE_H
#include <iostream>
#include "vector.h"
using namespace std;



// type redefinitions
typedef Vector<double> PositionVector;
typedef Vector<double> HeadingVector;



// "Pose" type definition
class Pose: public PositionVector
{
   public:

      // <public constructors>
      Pose(const PositionVector pv  = PositionVector(),
           const double         deg = 0.0f);
      Pose(const PositionVector pv, const HeadingVector hv);
      Pose(const Pose &pose);

      // <public destructors>
      virtual ~Pose();

      // <virtual public mutator functions>
      virtual bool setPosition(const double dx,
                               const double dy);
      virtual bool setPosition(const double dx,
                               const double dy,
                               const double dz);
      virtual bool setPosition(const PositionVector &pv);
      virtual bool setHeading(const double deg);
      virtual bool setHeading(const HeadingVector &hv);
      virtual bool rotateHeading(const double deg);

      // <public accessor functions>
      PositionVector getPosition()      const;
      double         getHeading()       const;
      HeadingVector  getHeadingVector() const;

      // <virtual public utility functions>
      virtual PositionVector getRelationshipTo(PositionVector pv);
      virtual void print();
      virtual void print(ostream &out);

      // <public friend functions>
      friend ostream& operator <<(ostream &out, const Pose &pose);

   protected:

      // <protected data members>
      double heading;
}; // Pose

#endif
