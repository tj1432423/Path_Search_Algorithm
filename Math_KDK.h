#ifndef MATH_KDK_H
#define MATH_KDK_H



class Math_KDK{
public:
    template<class T>
    T abs(const T& a)
    {
        return a > 0 ? a : -a;
    }

};




#endif // MATH_KDK_H
