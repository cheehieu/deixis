// preprocessor directives
#ifndef TIME_H
#define TIME_H
#include <boost/date_time/posix_time/posix_time.hpp>



// time definitions
typedef boost::posix_time::ptime Time;
#define getTime() (boost::posix_time::microsec_clock::local_time())

#endif

