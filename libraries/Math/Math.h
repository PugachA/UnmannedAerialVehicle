#ifndef MATH_H
#define MATH_H

class Math
{
    public:
    template <typename T>
        static T limiter(T max, T min, T value)
        {
            return value >= max ? max : value <= min ? min : value;
        }
};        



#endif