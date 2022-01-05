#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include<iosfwd> // contains forward definitions for iostream objects
#include <cmath>

namespace rigid2d
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12){

        if (fabs(d2-d1)<=epsilon){
            return true;}
        
        return false;
        
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr double deg2rad(double deg)
    {
        return(deg * (PI/180.0));
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return(rad * (180.0/PI));
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens


    // static_assert(almost_equal(5, 5), "is_zero failed");
    // static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");
    // static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    // static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");
    // static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    /// \brief limits angle between +- PI
    /// \param rad - angle in radians
    /// \returns angle in radians
    double normalize_angle(double rad);


    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        double x ,y ;

        Vector2D();

        // \brief constructor for Vector2D with inputs
        Vector2D(double x, double y);

        /// \brief vector normalization
        /// \param v - the vector to normalise
        /// \returns the normalized vector
        Vector2D normalize(const Vector2D & v);

        /// \brief vector addition
        /// \param v - the vector to add
        /// \returns returns the same vector with new value
        Vector2D & operator+=(const Vector2D & v);

        /// \brief perform vector subtraction
        /// \param v - the vector to subtract
        /// \returns the same vector with new value
        Vector2D & operator-=(const Vector2D & v);

        /// \brief perform scalar multiplication on a vector
        /// \param s -  scalar 
        /// \returns the same vector with new value
        Vector2D & operator*=(const double s);

        /// \brief calculates the magnitude of vector
        /// \returns returns the magnitude value
        double magnitude();


        /// \brief Compute and the angle of a vector
        /// \returns returns angle in radians
        double angle();


    };

    /// \brief two vector addition
    /// \param lhs - the left vector 
    /// \param rhs - the right vector 
    /// \returns returns rhs with new value
    Vector2D & operator+(const Vector2D & lhs, Vector2D & rhs);

    /// \brief two vector subtraction
    /// \param lhs - the left vector 
    /// \param rhs - the right vector 
    /// \returns returns rhs with new value
    Vector2D & operator-(const Vector2D & lhs, Vector2D & rhs);

    /// \brief two vector addition
    /// \param lhs - the left vector 
    /// \param s - scalar  
    /// \returns returns lhs with new value
    Vector2D & operator*(Vector2D & lhs, const double s);


    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output toc
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered as [xcomponent, ycomponent]
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    std::istream & operator>>(std::istream & is, Vector2D & v);


    /// \brief A 2-Dimensional Twist:Angular velocity in z direction and linear velocity in x and y direction
    struct Twist2D{
        double linear_vx=0.0;
        double linear_vy=0.0;
        double angle_z=0.0;

    };

    /// \brief should print a human readable version of the twist:
    /// An example output:
    /// angular_z (degrees): 90 linear_vx: 3 linear_vy: 5
    /// \param os - an output stream
    /// \param tf - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & twist);

    /// \brief Read a twist from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (angular_z, linear_vx, linear_vy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Twist2D & twist);



    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    public:
        double angle, dx, dy; 
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(const Vector2D & trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(const Vector2D & trans, double radians);

        

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;


        /// \brief compute the transformation corresponding to a rigid body following a constant twist 
        ///         for one time unit 
        /// \param twist -constant twist 
        /// \returns the new tranformation in origianl frame
        Transform2D integrateTwist(const Twist2D & twist);

        
    
        /// \brief convert a twist to a different reference frame using the adjoint
        /// \param tf - transform and twist 
        /// \returns a Twist in the new coordinate system
        Twist2D gettwist_inframe(const Twist2D & twist,const Transform2D & tf);

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \returns a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        
        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);


        /// \brief compute transformation corresponding to a rigid body
        /// \param tw - Twist
        /// \return the transformation corresponding to a rigid body for one time unit
        // Transform2D integrateTwist(const Twist2D & tw) const;

    private:
        
        Transform2D(double angle, double dx, double dy);
    
    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

}
#endif
