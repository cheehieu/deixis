// preprocessor directives
#ifndef VECTOR_H
#define VECTOR_H
#include <iostream>
#include <math.h>
#include "angles.h"
#include "utils.h"
using namespace std;



// forward declaration of class and friend functions
template <class T> class Vector;
template <class T> ostream&  operator <<(ostream &out, const Vector<T> &v);
template <class T> Vector<T> operator *(const double scalar,
                                        const Vector<T> &v);
template <class T> Vector<T> operator *(const Vector<T> &v,
                                        const double scalar);
template <class T> Vector<T> operator /(const Vector<T> &v,
                                        const double scalar);
template <class T> Vector<T> unit(const Vector<T> &v);
template <class T> Vector<T> crossProduct(const Vector<T> &v1,
                                          const Vector<T> &v2);
template <class T> double    dotProduct(const Vector<T> &v1,
                                        const Vector<T> &v2);
template <class T> double    angleBetween(const Vector<T> &v1,
                                          const Vector<T> &v2);
template <class T> double    angleBetween(const Vector<T> &v,
                                          const double     deg);
template <class T> double    angleBetween(const double     deg,
                                          const Vector<T> &v);



// "Vector" type definition
template <class T>
class Vector
{
   public:

      // <public data members>
      T x, y, z;

      // <public constructors>
      Vector(const T dx = 0.0f,
             const T dy = 0.0f,
             const T dz = 0.0f);
      Vector(const Vector<T> &v);

      // <public destructors>
      virtual ~Vector();

      // <virtual public mutator functions>
      virtual bool set(const T dx, const T dy);
      virtual bool set(const T dx, const T dy, const T dz);
      virtual bool set(const Vector<T> &v);
      virtual bool rotate(const double deg);

      // <public mutator functions>
      bool setPolar(const double mag, const double deg);
      bool setMagnitude(const double mag);
      bool setAngle(const double deg);
      bool setDiff(const Vector<T> &src,
                   const Vector<T> &dst);
      bool setPerp();
      bool setAvg(const Vector<T> vectors[], const int n_vectors);
      bool normalize();

      // <public utility functions>
      double    magnitude()                   const;
      double    angle()                       const;
      Vector<T> rotated(const double deg)     const;
      Vector<T> perp()                        const;
      double    perpDot(const Vector<T> &v) const;

      // <virtual public utility functions>
      void print();
      void print(ostream &out);

      // <virtual public overloaded operators>
      virtual Vector<T>& operator  =(const Vector<T> &v);
      virtual Vector<T>  operator  +(const Vector<T> &v);
      virtual Vector<T>  operator  -(const Vector<T> &v);
      virtual Vector<T>  operator  -();
      virtual Vector<T>& operator +=(const Vector<T> &v);
      virtual Vector<T>& operator -=(const Vector<T> &v);
      virtual Vector<T>& operator *=(const double  scalar);
      virtual Vector<T>& operator /=(const double  scalar);
      virtual bool       operator ==(const Vector<T> &v);
      virtual bool       operator !=(const Vector<T> &v);

      // <public friend functions>
      friend ostream&  operator << <>(ostream &out, const Vector<T> &v);
      friend Vector<T> operator * <>(const double       scalar,
                                     const Vector<T> &v);
      friend Vector<T> operator * <>(const Vector<T> &v,
                                     const double       scalar);
      friend Vector<T> operator / <>(const Vector<T> &v,
                                     const double       scalar);
      friend Vector<T> unit <>(const Vector<T> &v);
      friend Vector<T> crossProduct <>(const Vector<T> &v1,
                                       const Vector<T> &v2);
      friend double    dotProduct <>(const Vector<T> &v1,
                                     const Vector<T> &v2);
      friend double    angleBetween <>(const Vector<T> &v1,
                                       const Vector<T> &v2);
      friend double    angleBetween <>(const Vector<T> &v,
                                       const double     deg);
      friend double    angleBetween <>(const double     deg,
                                       const Vector<T> &v);
};  // Vector



// <public constructors>

//
// Vector(dx, dy, dz)
// Last modified: 21Apr2008
//
// Default constructor that initializes
// the vector to the parameterized values.
//
// Returns:     <none>
// Parameters:
//      dx      in      the initial x-coordinate of the Vector (default 0)
//      dy      in      the initial y-coordinate of the Vector (default 0)
//      dz      in      the initial z-coordinate of the Vector (default 0)
//
template <class T>
Vector<T>::Vector(const T dx, const T dy, const T dz)
   : x(dx), y(dy), z(dz)
{
}  // Vector(const T, const T, const T)



//
// Vector(v)
// Last modified: 21Apr2008
//
// Copy constructor that copies the contents of
// the parameterized vector into this vector.
//
// Returns:     <none>
// Parameters:
//      v       in/out      the circle being copied
//
template <class T>
Vector<T>::Vector(const Vector<T> &v)
   : x(v.x), y(v.y), z(v.z)
{
}  // Vector(const Vector<T> &)



// <public destructors>

//
// ~Vector()
// Last modified: 12Feb2008
//
// Destructor that clears this vector.
//
// Returns:     <none>
// Parameters:  <none>
//
template <class T>
Vector<T>::~Vector()
{
}  // ~Vector()



// <virtual public mutator functions>

//
// bool set(dx, dy)
// Last modified: 12Feb2008
//
// Attempts to set the xy-coordinates to the corresponding
// parameterized values, returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      dx      in      the x-coordinate to be set to (default 0)
//      dy      in      the y-coordinate to be set to (default 0)
//
template <class T>
bool Vector<T>::set(const T dx, const T dy)
{
   x = dx;
   y = dy;
   return true;
}  // set(const T, const T)



//
// bool set(dx, dy, dz)
// Last modified: 12Feb2008
//
// Attempts to set the xyz-coordinates to the corresponding
// parameterized values, returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      dx      in      the x-coordinate to be set to
//      dy      in      the y-coordinate to be set to
//      dz      in      the z-coordinate to be set to
//
template <class T>
bool Vector<T>::set(const T dx, const T dy, const T dz)
{
   z = dz;
   return set(dx, dy);
}  // set(const T, const T, const T)



//
// bool set(v)
// Last modified: 21Feb2008
//
// Attempts to set the xyz-coordinates based upon the parameterized
// vector, returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      v       in      the Vector to be set to
//
template <class T>
bool Vector<T>::set(const Vector<T> &v)
{
   return set(v.x, v.y, v.z);
}  // set(const Vector<T> &)



//
// bool rotate(deg)
// Last modified: 12Feb2008
//
// Rotates the vector about the z-axis
// based on the parameterized angle (in degrees),
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      deg     in      the rotation angle (in degrees)
//
template <class T>
bool Vector<T>::rotate(const double deg)
{
   if (magnitude() == 0.0f) return false;
   double rad = degreesToRadians(deg);
   return set(double(x) * cos(rad) - double(y) * sin(rad),
              double(x) * sin(rad) + double(y) * cos(rad));
}  // rotate(const double)



// <public mutator functions>

//
// bool setPolar(mag, deg)
// Last modified: 21Apr2008
//
// Attempts to set the vector based on the parameterized polar coordinates,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      mag     in      the magnitude to be set to
//      deg     in      the angle (in degrees) to be set to
//
template <class T>
bool Vector<T>::setPolar(const double mag, const double deg)
{
   double rad = degreesToRadians(deg);
   return set(mag * cos(rad), mag * sin(rad));
}  // setPolar(const double, const double)



//
// bool setMagnitude(mag)
// Last modified: 12Feb2008
//
// Attempts to set the normal (magnitude) to the parameterized magnitude,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      mag     in      the magnitude to be set to (default 1)
//
template <class T>
bool Vector<T>::setMagnitude(const double mag)
{
   if (!normalize()) return false;
   return set(double(x) * mag, double(y) * mag, double(z) * mag);
}  // setMagnitude(const double)



//
// bool setAngle(deg)
// Last modified: 12Feb2008
//
// Attempts to set the angle to the parameterized angle (in degrees),
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      deg     in      the angle to be set to
//
template <class T>
bool Vector<T>::setAngle(const double deg)
{
	return setPolar(magnitude(), deg);
}  // setAngle(const double)



//
// bool setDiff(src, dst)
// Last modified: 23Apr2008
//
// Attempts to set the vector to the difference from the
// parameterized source to the parameterized destination,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      src     in      the source vector
//      dst     in      the destination vector
//
template <class T>
bool Vector<T>::setDiff(const Vector<T> &src, const Vector<T> &dst)
{
   return set(dst.x - src.x, dst.y - src.y, dst.z - src.z);
}  // setDiff(const Vector<T> &, const Vector<T> &)



//
// bool setPerp()
// Last modified: 12Feb2008
//
// Attempts to set this vector to its perpendicular vector,
// returning true if successful, false otherwise.
//
// Returns:     <none>
// Parameters:  <none>
//
template <class T>
bool Vector<T>::setPerp()
{
   T tmp =  x;
   x     = -y;
   y     = tmp;
   return true;
}  // setPerp()



//
// bool setAvg(vectors, n_vectors)
// Last modified: 25Aug2009
//
// Attempts to set the vector based on the average
// of the parameterized vectors, returning true
// if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      vectors    in/out  the vectors to be averaged
//      n_vectors   in      the number of vectors to be averaged
//
template <class T>
bool Vector<T>::setAvg(const Vector<T> vectors[], const int n_vectors)
{
   if (n_vectors <= 0) return false;
   for (int i = 0; i < n_vectors; i++) *this += vectors[i];
   x /= (T)n_vectors;
   y /= (T)n_vectors;
   z /= (T)n_vectors;
   return true;
}  // setAvg(const Vector<T> [], const int)



//
// bool normalize()
// Last modified: 21Apr2008
//
// Attempts to adjust the vector to unit length,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:  <none>
//
template <class T>
bool Vector<T>::normalize()
{
   double mag = magnitude();
   if (mag == 0.0f) return false;   // does nothing to zero vectors
   return set(double(x) / mag, double(y) / mag, double(z) / mag);
}  // normalize()



// <public utility functions>

//
// double magnitude() const
// Last modified: 21Apr2008
//
// Returns the magnitude (normal) of this vector.
//
// Returns:     the magnitude (normal) of this vector
// Parameters:  <none>
//
template <class T>
double Vector<T>::magnitude() const
{
   return sqrt(pow(double(x), double(2.0f)) +
               pow(double(y), double(2.0f)) +
               pow(double(z), double(2.0f)));
}  // magnitude()



//
// double angle() const
// Last modified: 11Dec2008
//
// Returns the angle of this vector.
//
// Returns:     the angle of this Vector
// Parameters:  <none>
//
template <class T>
double Vector<T>::angle() const
{
   if ((x == 0.0f) && (y == 0.0f) && (z == 0.0f)) return 0.0f;
   return radiansToDegrees(atan2(y, x));
   return sign(y) *
          radiansToDegrees(acos(dotProduct(unit(*this), unit(Vector(1.0f)))));
}  // angle()



//
// Vector<T> rotated(deg) const
// Last modified: 18Apr2008
//
// Returns this vector rotated about the z-axis
// based on the parameterized angle (in degrees).
//
// Returns:     returns this vector rotated about the z-axis
// Parameters:
//      deg     in      the rotation angle (in degrees)
//
template <class T>
Vector<T> Vector<T>::rotated(const double deg) const
{
   Vector<T> v(*this);
   v.rotate(deg);
   return v;
}  // rotated(const double)



//
// void perp() const
// Last modified: 12Feb2008
//
// Returns the perpendicular vector of this vector.
//
// Returns:     the perpendicular vector of this vector
// Parameters:  <none>
//
template <class T>
Vector<T> Vector<T>::perp() const
{
   Vector<T> tmp = *this;
   tmp.setPerp();
   return tmp;
}  // perp()



//
// double perpDot(v) const
// Last modified: 12Feb2008
//
// Returns the dot product of the perpendicular vector
// of this vector and the parameterized vector.
//
// Returns:     the dot product of the perpendicular vector
//              of this vector and the parameterized vector
// Parameters:
//      v       in/out  the vector to be dotted with
//
template <class T>
double Vector<T>::perpDot(const Vector<T> &v) const
{
   return double(x) * double(v.x) - double(y) * double(v.y);
}  // perpDot(const Vector<T> &)



// <virtual public utility functions>

//
// void print()
// Last modified: 21Feb2008
//
// Outputs this vector to the screen.
//
// Returns:     <none>
// Parameters:  <none>
//
template <class T>
void Vector<T>::print()
{
   print(cout);
}  // print()



//
// void print(out)
// Last modified: 21Feb2008
//
// Outputs this vector to the parameterized output file stream.
//
// Returns:     <none>
// Parameters:
//      out     in/out  the output files stream
//
template <class T>
void Vector<T>::print(ostream &out)
{
   out << *this << endl;
}  // print(ostream &)



// <virtual public overloaded operators>

//
// Vector<T>& =(v)
// Last modified: 21Apr2008
//
// Copies the contents of the parameterized vector into this vector.
//
// Returns:     this vector
// Parameters:
//      v       in/out  the vector being copied
//
template <class T>
Vector<T>& Vector<T>::operator =(const Vector<T> &v)
{
   set(v.x, v.y, v.z);
   return *this;
}  // =(const Vector<T> &)



//
// Vector<T> +(v)
// Last modified: 21Apr2008
//
// Calculates the sum of the contents of
// the parameterized vector and this vector.
//
// Returns:     the vector sum
// Parameters:
//      v       in/out  the vector being added
//
template <class T>
Vector<T> Vector<T>::operator +(const Vector<T> &v)
{
   return Vector(this->x + v.x, this->y + v.y, this->z + v.z);
}  // +(const Vector<T> &)



//
// Vector<T> -(v)
// Last modified: 17Feb2008
//
// Calculates the difference of the contents of
// the parameterized vector and this vector.
//
// Returns:     this vector difference
// Parameters:
//      v       in/out  the Vector being subtracted
//
template <class T>
Vector<T> Vector<T>::operator -(const Vector<T> &v)
{
   return *this + -const_cast<Vector<T> &>(v);
}  // -(const const Vector<T> &)



//
// Vector<T> -()
// Last modified: 17Feb2008
//
// Returns the negation of the this vector.
//
// Returns:     the negation of the this vector
// Parameters:  <none>
//
template <class T>
Vector<T> Vector<T>::operator -()
{
   return -1.0f * *this;
}  // -()



//
// Vector<T>& +=(v)
// Last modified: 17Feb2008
//
// Adds the contents of the parameterized vector to this vector.
//
// Returns:     this vector
// Parameters:
//      v       in/out  the vector being added
//
template <class T>
Vector<T>& Vector<T>::operator +=(const Vector<T> &v)
{
   return *this = *this + v;
}  // +=(const Vector<T> &)



//
// Vector<T>& -=(v)
// Last modified: 17Feb2008
//
// Subtracts the contents of the parameterized vector from this vector.
//
// Returns:     this vector
// Parameters:
//      v       in/out  the vector being subtracted
//
template <class T>
Vector<T>& Vector<T>::operator -=(const Vector<T> &v)
{
   return *this = *this - v;
}  // +=(Vector<T> &)



//
// Vector<T>& *=(v)
// Last modified: 17Feb2008
//
// Multiplies this vector by the parameterized scalar.
//
// Returns:     this vector
// Parameters:
//      scalar  in      the vector multiplier
//
template <class T>
Vector<T>& Vector<T>::operator *=(const double scalar)
{
   return *this = *this * scalar;
}  // *=(const double)



//
// Vector<T>& /=(v)
// Last modified: 19Apr2008
//
// Divides this vector by the parameterized scalar.
//
// Returns:     this vector
// Parameters:
//      scalar  in      the vector divisor
//
template <class T>
Vector<T>& Vector<T>::operator /=(const double scalar)
{
   return *this = *this / scalar;
}  // /=(const double)



//
// bool ==(v)
// Last modified: 21Apr2008
//
// Compares this vector to the parameterized vector,
// returning true if they are the same, false otherwise.
//
// Returns:     true if the vector are the same, false otherwise
// Parameters:
//      v       in/out  the vector to compare to
//
template <class T>
bool Vector<T>::operator ==(const Vector<T> &v)
{
   return (x == v.x) && (y == v.y) && (z == v.z);
}  // ==(const Vector<T> &)



//
// bool !=(v)
// Last modified: 17Feb2008
//
// Compares this vector to the parameterized vector,
// returning true if they are not the same, false otherwise.
//
// Returns:     true if the vectors are not the same, false otherwise
// Parameters:
//      v       in/out  the vector to compare to
//
template <class T>
bool Vector<T>::operator !=(const Vector<T> &v)
{
   return !(*this == v);
}  // ==(const Vector<T> &)



// <public friend functions>

//
// ostream& <<(out, v)
// Last modified: 12Feb2008
//
// Outputs the parameterized vector to the parameterized output file stream.
//
// Returns:     the output file stream
// Parameters:
//      out     in/out  the output files stream
//      v       in/out  the vector to output
//
template <class T>
ostream& operator <<(ostream &out, const Vector<T> &v)
{
   out << "[" << v.x << ", " << v.y << ", " << v.z << "]";
   return out;
}  // <<(ostream &, const Vector<T> &)



//
// Vector<T> *(scalar, v)
// Last modified: 12Feb2008
//
// Returns the parameterized vector multiplied by the parameterized scalar.
//
// Returns:     the parameterized vector multiplied by the parameterized scalar
// Parameters:
//      scalar  in      the Vector multiplier
//      v       in/out  the Vector to be negated
//
template <class T>
Vector<T> operator *(const double scalar, const Vector<T> &v)
{
   return Vector<T>(scalar * v.x, scalar * v.y, scalar * v.z);
}  // *(const double, const Vector<T> &)



//
// Vector<T> *(v, scalar)
// Last modified: 12Feb2008
//
// Returns the parameterized vector multiplied by the parameterized scalar.
//
// Returns:     the parameterized vector multiplied by the parameterized scalar
// Parameters:
//      v       in/out  the Vector to be negated
//      scalar  in      the Vector multiplier
//
template <class T>
Vector<T> operator *(const Vector<T> &v, const double scalar)
{
   return scalar * v;
}  // *(const Vector<T> &, const double)



//
// Vector<T> /(v, scalar)
// Last modified: 18Apr2008
//
// Returns the parameterized vector divided by the parameterized scalar.
//
// Returns:     the parameterized vector divided by the parameterized scalar
// Parameters:
//      v       in/out  the Vector to be negated
//      scalar  in      the Vector multiplier
//
template <class T>
Vector<T> operator /(const Vector<T> &v, const double scalar)
{
   return Vector<T>(v.x / scalar, v.y / scalar, v.z / scalar);
}  // /(const Vector<T> &, const double)



//
// Vector<T> unit(v)
// Last modified: 12Feb2008
//
// Returns the unit vector of the parameterized vector.
//
// Returns:     the unit vector of the parameterized vector
// Parameters:
//      v       in/out  the vector to be unitized
//
template <class T>
Vector<T> unit(const Vector<T> &v)
{
   Vector<T> tmp = v;
   tmp.normalize();
   return tmp;
}  // unit(const Vector<T> &)



//
// Vector<T> crossProduct(v1, v2)
// Last modified: 12Feb2008
//
// Returns the cross product of the parameterized vectors.
//
// Returns:     the cross product of the parameterized vectors
// Parameters:
//      v1      in/out  the first vector
//      v2      in/out  the second vector
//
template <class T>
Vector<T> crossProduct(const Vector<T> &v1, const Vector<T> &v2)
{
   return Vector<T>(v1.y * v2.z - v2.y * v1.z,
                    v1.z * v2.x - v2.z * v1.x,
                    v1.x * v2.y - v2.x * v1.y);
}  // crossProduct(const Vector<T> &, const Vector<T> &)



//
// Vector<T> dotProduct(v1, v2)
// Last modified: 12Feb2008
//
// Returns the dot product of the parameterized vectors.
//
// Returns:     the dot product of the parameterized vectors
// Parameters:
//      v1      in/out  the first vector
//      v2      in/out  the second vector
//
template <class T>
double dotProduct(const Vector<T> &v1, const Vector<T> &v2)
{
   return double(v1.x) * double(v2.x) +
          double(v1.y) * double(v2.y) +
          double(v1.z) * double(v2.z);
}  // dotProduct(const Vector<T> &, const Vector<T> &)



//
// double angleBetween(v1, v2)
// Last modified: 12Feb2008
//
// Returns the angle between the parameterized vectors.
//
// Returns:     the angle between the parameterized vectors
// Parameters:
//      v1      in/out  the first vector
//      v2      in/out  the second vector
//
template <class T>
double angleBetween(const Vector<T> &v1, const Vector<T> &v2)
{
   return scaleDegrees(v1.angle() - v2.angle());
}  // angleBetween(const Vector<T> &, const Vector<T> &)



//
// double angleBetween(v, deg)
// Last modified: 20Feb2008
//
// Returns the angle between the parameterized vector and angle (in degrees).
//
// Returns:     the angle between the parameterized vector and angle
// Parameters:
//      v       in/out  the vector
//      deg     in/out  the angle (in degrees)
//
template <class T>
double angleBetween(const Vector<T> &v, const double deg)
{
   return scaleDegrees(v.angle() - deg);
}  // angleBetween(const Vector<T> &, const double)



//
// double angleBetween(deg, v)
// Last modified: 20Feb2008
//
// Returns the angle between the parameterized angle (in degrees) and vector.
//
// Returns:     the angle between the parameterized angle and vector
// Parameters:
//      deg     in/out  the angle (in degrees)
//      v       in/out  the vector
//
template <class T>
double angleBetween(const double deg, const Vector<T> &v)
{
   return scaleDegrees(deg - v.angle());
}  // angleBetween(const double, const Vector<T> &)

#endif
