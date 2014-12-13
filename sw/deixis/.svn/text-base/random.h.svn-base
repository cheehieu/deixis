// preprocessor directives
#ifndef RANDOM_H
#define RANDOM_H
#include <iostream>
#include <stdlib.h>



// floating-point definitions
#ifndef F_EPSILON
#define F_EPSILON (5.96046447753906E-8)
#endif



// function definitions
#define randSign() ((rand() % 2) ? -1 : 1)



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
double frand(const double min = 0.0f, const double max = 1.0f + F_EPSILON);



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
int irand(const int min = 0, const int max = 2);

#endif

