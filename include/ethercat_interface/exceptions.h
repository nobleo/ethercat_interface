#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

#include <exception>

class SocketError: public std::exception
{
    virtual const char* what() const throw()
    {
        return "No socket connection on the provided interface. "
               "If your interface is defined correctly, "
               "try excecuting the following command: "
               "sudo setcap cap_net_raw+ep on your executable";
    }
} socketerror;

class NoSlaveError: public std::exception
{
    virtual const char* what() const throw()
    {
        return "No slaves found";
    }
} noslaveerror;

#endif // EXCEPTIONS_H
