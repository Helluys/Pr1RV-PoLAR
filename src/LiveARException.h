#ifndef LIVEAREXCPETION_H
#define LIVEAREXCPETION_H

#include <stdexcept>

class LiveARException : public std::runtime_error
{
    public:
        enum Type
        {
            UNKNOWN,
            INVALID_CAMERA
        } type;

        LiveARException(const std::string &what = "", Type t = UNKNOWN) : std::runtime_error(what.c_str()), type(t)
        {}
};

#endif
