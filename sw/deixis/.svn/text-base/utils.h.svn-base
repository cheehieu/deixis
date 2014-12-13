// preprocessor directives
#ifndef UTILS_H
#define UTILS_H



// floating-point definitions
#ifndef F_EPSILON
#define F_EPSILON (5.96046447753906E-8)
#endif



// function definitions
#define sign(x) (((x) < 0) ? -1 : 1)
#define clip(x, x_min, x_max) (std::min(std::max((x), (x_min)), (x_max)))
#define isBetween(x, x_min, x_max) (((x) >= (x_min)) && ((x) <= (x_max)))
#define fisZero(x) (isBetween((x), -F_EPSILON, F_EPSILON))

#endif
