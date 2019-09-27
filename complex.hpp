#ifndef _COMPLEX_H_P_P_
#define _COMPLEX_H_P_P_

#include <cmath>

/**********************************/
/*      Forward Declarations      */
/**********************************/
class complex;

constexpr complex operator+(complex const& c1, complex const& c2) noexcept;
template<typename T> constexpr complex operator+(complex const& c, T const& t) noexcept;
template<typename T> constexpr complex operator+(T const& t, complex const& c) noexcept;

constexpr complex operator-(complex const& c1, complex const& c2) noexcept;
template<typename T> constexpr complex operator-(complex const& c, T const& t) noexcept;
template<typename T> constexpr complex operator-(T const& t, complex const& c) noexcept;

constexpr complex operator*(complex const& c1, complex const& c2) noexcept;
template<typename T> constexpr complex operator*(complex const& c, T const& t) noexcept;
template<typename T> constexpr complex operator*(T const& t, complex const& c) noexcept;

constexpr complex operator/(complex const& c1, complex const& c2) noexcept;
template<typename T> constexpr complex operator/(complex const& c, T const& t) noexcept;
template<typename T> constexpr complex operator/(T const& t, complex const& c) noexcept;


/*************************/
/*      Definitions      */
/*************************/
struct complex
{
    float real, imaginary;

    constexpr complex operator-() const noexcept
    {
        return { -real, -imaginary };
    }

    template<typename T>
    constexpr complex& operator+=(T const& c) noexcept
    {
        return *this = *this + c;
    }

    template<typename T>
    constexpr complex& operator-=(T const& c) noexcept
    {
        return *this = *this - c;
    }

    template<typename T>
    constexpr complex& operator*=(T const& c) noexcept
    {
        return *this = *this * c;
    }
    
    template<typename T>
    constexpr complex& operator/=(T const& c) noexcept
    {
        return *this = *this / c;
    }

    constexpr float magnitude() const noexcept
    {
        return sqrt(real * real + imaginary * imaginary);
    }
};

/**********************/
/*      Addition      */
/**********************/

constexpr complex operator+(complex const& c1, complex const& c2) noexcept
{
    return { c1.real + c2.real, c1.imaginary + c2.imaginary };
}

template<typename T>
constexpr complex operator+(complex const& c, T const& t) noexcept
{
    return c + complex{ t, 0 };
}

template<typename T>
constexpr complex operator+(T const& t, complex const& c) noexcept
{
    return c + t;
}


/***********************/
/*     Subtraction     */
/***********************/

constexpr complex operator-(complex const& c1, complex const& c2) noexcept
{
    return { c1.real - c2.real, c1.imaginary - c2.imaginary };
}

template<typename T>
constexpr complex operator-(complex const& c, T const& t) noexcept
{
    return c - complex{ t, 0 };
}

template<typename T>
constexpr complex operator-(T const& t, complex const& c) noexcept
{
    return complex{ t, 0 } - c;
}

/**********************/
/*   Multiplication   */
/**********************/

constexpr complex operator*(complex const& c1, complex const& c2) noexcept
{
    return { c1.real * c2.real - c1.imaginary * c2.imaginary, 
             c1.imaginary * c2.real + c2.imaginary * c1.real };
}

template<typename T>
constexpr complex operator*(complex const& c, T const& t) noexcept
{
    return c * complex{ t, 0 };
}

template<typename T>
constexpr complex operator*(T const& t, complex const& c) noexcept
{
    return c * t;
}

/**********************/
/*      Division      */
/**********************/

constexpr complex operator/(complex const& c1, complex const& c2) noexcept
{
    return { (c1.real * c2.real - c1.imaginary * c2.imaginary) / (c2.real * c2.real + c2.imaginary + c2.imaginary), 
             (c1.imaginary * c2.real - c2.imaginary * c1.real) / (c2.real * c2.real + c2.imaginary + c2.imaginary) };
}

template<typename T>
constexpr complex operator/(complex const& c, T const& t) noexcept
{
    return c / complex{ t, 0 };
}

template<typename T>
constexpr complex operator/(T const& t, complex const& c) noexcept
{
    return complex{ t, 0 } / c;
}

/***********************************/
/*      Mathmatical Functions      */
/***********************************/

constexpr complex log(complex const& c) noexcept
{
    return { c.magnitude(), atan2f(c.imaginary, c.real) };
}

#endif /* _COMPLEX_H_P_P_ */