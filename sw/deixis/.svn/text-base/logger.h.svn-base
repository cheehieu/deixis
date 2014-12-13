// preprocessor directives
#ifndef LOGGER_H
#define LOGGER_H
#include <fstream>
#include <iostream>
#include "mutex.h"
using namespace std;



// filename definitions
#define FILENAME_PREFIX    ("data/data")
#define FILENAME_EXTENSION ("csv")



//
template <class T>
struct Logger: Mutex
{

  // <data members>
  ofstream data_file;
  T        data;



  // <constructors>
  Logger(char* filename = NULL);



  // <destructors>
  ~Logger();



  // <utility functions>
  bool open(char* filename = NULL);
  bool close();
  bool isOpen();
  bool log();
};// Logger



// <constructors>

//
template <class T>
Logger<T>::Logger(char* filename): data()
{
  if (filename != NULL) open(filename);
} // Logger()



// <destructors>

//
template <class T>
Logger<T>::~Logger()
{
  close();
} // ~Logger()



// <utility functions>

//
template <class T>
bool Logger<T>::open(char* filename)
{
  lock();
  if (data_file.is_open()) data_file.close();
  if (filename != NULL)    data_file.open(filename);
  else
  {
    filename = new char[50];

    ifstream fin;
    int      id = 0;
    while (true)
    {
      sprintf(filename, ((id < 10) ? "%s0%d.%s" : "%s%d.%s"),
              FILENAME_PREFIX, id, FILENAME_EXTENSION);
      fin.open(filename);
      if (fin.is_open())
      {
        fin.close();
        ++id;
      }
      else break;
    }

    data_file.open(filename);
    delete [] filename;
  }
  bool success = data_file.is_open();
  unlock();
  return success;
} // open(char*)



//
template <class T>
bool Logger<T>::close()
{
  lock();
  data_file.close();
  bool success = !data_file.is_open();
  unlock();
  return success;
} // close()



//
template <class T>
bool Logger<T>::isOpen()
{
  lock();
  bool success = data_file.is_open();
  unlock();
  return success;
} // isOpen()



//
template <class T>
bool Logger<T>::log()
{
  static bool header_written = false;
  lock();
  if (!header_written) header_written = data.logHeader(data_file);
  bool success = header_written && data.log(data_file);
  unlock();
  return success;
} // log()

#endif

