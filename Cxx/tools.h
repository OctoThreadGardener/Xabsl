// DO NOT EDIT
/*****************************
	Adapts xabsl::Engine to the target platform.
*****************************/

#ifndef TOOLS_H
#define TOOLS_H

#include <fstream>
#include <sys/timeb.h>
#include <iostream>

#include "../Xabsl/XabslEngine/XabslEngine.h"

using namespace std;

/* An error handling class derived from xabsl::ErrorHandler */
class MyErrorHandler : public xabsl::ErrorHandler
{
public:
    MyErrorHandler()
    { }

    virtual void printError(const char *text)
    { cout << "error: " << text << endl; }

    virtual void printMessage(const char *text)
    { cout << text << endl; }
};

/* A file access class derived from xabsl::InputSource */
/* It gives the engine a read access to the XABSL code */
class MyFileInputSource : public xabsl::InputSource
{
public:
    MyFileInputSource(const char *fileName)
            : file(NULL), theChar(' ')
    { strcpy(filename, fileName); }

    ~MyFileInputSource()
    {
        if (file != NULL)
        { delete file; }
    }

    virtual bool open()
    {
        cout << "Opening file: " << filename << endl;
        file = new ifstream(filename, ios::in);
        return ((file != 0) && (!file->fail()));
    }

    virtual void close()
    {
        if (file != NULL)
        { delete file; }
        file = NULL;
    }

    virtual double readValue()
    {
        char buf[20];
        readFromFile(buf);
        return atof(buf);
    }

    virtual bool readString(char *dest, int maxLength)
    {
        readFromFile(dest);
        return true;
    }

private:
    char filename[200];
    std::ifstream *file;
    char theChar;

    void readFromFile(char *value)
    {
        while (!file->eof() && isWhitespace())
        {
            if (theChar == '/')
            {
                while (!file->eof() && theChar != '\n') file->read(&theChar, 1);
            }
            file->read(&theChar, 1);
        }

        while (!file->eof() && !isWhitespace())
        {
            *value++ = theChar;
            if (!file->eof()) file->read(&theChar, 1);
        }

        *value = 0;
    }

    bool isWhitespace()
    {
        return theChar == ' ' || theChar == '/' || theChar == '\n' || theChar == '\r' || theChar == '\t';
    }
};

static unsigned getSystemTime()
{
    timeb sysTime;
    ftime(&sysTime);
    return (sysTime.time * 1000 + sysTime.millitm);
}

#endif
