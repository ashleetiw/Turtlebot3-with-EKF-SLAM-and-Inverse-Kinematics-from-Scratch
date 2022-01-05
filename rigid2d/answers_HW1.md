# Rigid 2D transformation Library
A library for handling transformations in SE(2).

# Conceptual Questions
1. What is the difference between a class and a struct in C++?
  The default access mode and default inheritance mode are public for struct whereas class is private by default.
  
  
2. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specic C++ core guidelines in your answer)?
 1.)Use class if the class has an invariant; use struct if the data members can vary independently.
   Using Transform2D as a class alerts the programmer to the need for an invariant.In Vector2D ,struct members are independent of each other.
 2.)Use class rather than struct if any member is non-public
   Notice that vector2D is basically holding data whereas Transform2D have more functionality included.
  A good guideline is to use structs as that only hold data and use classes for when more functionality (member functions) is required.(https://stackoverflow.com/questions/15572665/c-structs-with-member-functions-vs-classes-with-public-variables)
   
	
3. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
  By default, declare single-argument constructors explicit
  Adding explicit before the construction ensures that during overloading there is no implicit conversion ie no unintended typcasting 


4. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   - Which of the methods would you implement and why?
   
   1.) Create a standalone function ( not member of any struct or class)
      It is easy to implement but accoriding to c++ guidelines things that have an invariant or like a logical condition they need to fulfil should be declared in a class or struct.
   
   
   2.)Declaring it within a class: Create a constructor in Transform 2D class which takes an arugment vector2D and returns normalise vector2d
   	[rigid2d::Vector2D rigid2d::Transform2D::normalize(rigid2d::Vector2D & v);]
   	-Implementation in a class means it is by default private over access but prone to errros because single arguments can have implicit conversion.
   	
   
   3.) Implementing  it as a member function in struct Vector2D.
   	[rigid2d::Vector2D rigid2d::Vector2D::normalize(const rigid2d::Vector2D v)]
   	-more secure as the original vector is getting not changing by using 'const'.
   	Considering the fact Vector2D is already declared as a struct , this method is easy to implement 
   	
     I would implement 3rd method because accroding to c++ guildelines use struct if the data members can vary independently.This method is easily readable and offers the security of never altering vector2d v
    
   
5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not (Refer to C++ core guidelines in your answer)?
   A const member function guarantees it will not modify the object or call any non-const member functions
   Transform2D::inv() is intended to not modify  any data members of the object that calls it.Transform2D::inv()  creates a new Transform2D which is returned later.The original transform  does not get   modified 
   Transform2D::operator *= is inteneted to manipulate the tranfrom it itself (t=t*r imples t=t*r).That is why const is not after becuasethe data of transform itself is getting changed 
