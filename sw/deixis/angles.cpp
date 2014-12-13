// preprocessor directives
#include "angles.h"



//
// double scaleDegrees(deg)
// Last modified: 12Feb2008
//
// Scales the parameterized angle (in degrees) to an angle [-180, 180].
//
// Returns:     the scaled angle (in degrees)
// Parameters:
//      deg     in      the angle (in degrees) to be scaled
//
double scaleDegrees(double deg)
{
   if        (deg >     0.0f)
      while ((deg >=  360.0f) || (deg >  180.0f)) deg -= 360.0f;
   else if   (deg <     0.0f)
      while ((deg <= -360.0f) || (deg < -180.0f)) deg += 360.0f;
   return deg;
}  // scaleDegrees(double)



//
// double scaleRadians(rad)
// Last modified: 12Feb2008
//
// Scales the parameterized angle (in radians) to an angle [-2 * PI, 2 * PI].
//
// Returns:     the scaled angle (in radians)
// Parameters:
//      rad     in      the angle (in radians) to be scaled
//
double scaleRadians(double rad)
{
   if        (rad >      0.0)
      while ((rad >=  TWO_PI) || (rad >  PI)) rad -= TWO_PI;
   else if   (rad <      0.0)
      while ((rad <= -TWO_PI) || (rad < -PI)) rad += TWO_PI;
   return rad;
}  // scaleRadians(double)



//
// double degreesToRadians(deg)
// Last modified: 12Feb2008
//
// Converts the parameterized angle (in degrees) to an angle in radians.
//
// Returns:     the converted angle (in radians)
// Parameters:
//      deg     in      the angle (in degrees) to converted to radians
//
double degreesToRadians(double deg)
{
   return scaleDegrees(deg) * PI_OVER_180;
}  // degreesToRadians(double)



//
// double radiansToDegrees(rad)
// Last modified: 12Feb2008
//
// Converts the parameterized angle (in radians) to an angle in degrees.
//
// Returns:     the converted angle (in degrees)
// Parameters:
//      rad     in      the angle (in radians) to converted to degrees
//
double radiansToDegrees(double rad)
{
   return scaleRadians(rad) / PI_OVER_180;
}  // radiansToDegrees(double)

