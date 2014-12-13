// preprocessor directives
#include "random.h"



//
// double frand(min, max)
// Last modified: 09Dec2008
//
// Returns a floating-point number [min, max].
//
// Returns:     a floating-point number [min, max]
// Parameters:
//      min     in      the minimum of the number being returned
//      max     in      the maximum of the number being returned
//
double frand(const double min, const double max)
{
   return min + (max - min) * double(rand()) / double(RAND_MAX);
}  // frand(const double, const double)



//
// int irand(min, max)
// Last modified: 20Feb2008
//
// Returns an integer number [min, max).
//
// Returns:     an integer number [min, max)
// Parameters:
//      min     in      the minimum of the number being returned
//      max     in      the maximum of the number being returned
//
int irand(const int min, const int max)
{
   return min + rand() % (max - min);
}  // irand()

