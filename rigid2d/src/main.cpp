/**********************************************  the file transforms vector and twists in different frames  *********************************************/
#include <iostream>  
#include <math.h>
#include "rigid2d/rigid2d.hpp"

int main(){
   
    using namespace rigid2d;

    std::cout << "Enter Tab Transform" << std::endl;
	Transform2D Tab;
	operator>>(std::cin, Tab);
	
	std::cout << "Enter Tbc Transform" << std::endl;
	Transform2D Tbc;
	operator>>(std::cin, Tbc);


    /*calculation for T_ba,T_ac,T_ca,T_cb */


    /* T_ba inverse of T_ab  */
    Transform2D Tba {Tab.inv()};

    /* T_cb inverse of T_bc */
    Transform2D Tcb {Tbc.inv()};
    
    Transform2D Tac=Tab*Tbc;

    /* T_ca inverse of T_ac */
    Transform2D Tca={Tac.inv()};

    /************************* reperesenting  vector 2D in different frames **********************************/
    std::cout << "Enter vector v :" << std::endl;
    Vector2D v;
    operator>>(std::cin, v);


    std::cout << "Enter frame {a,b or c} in which vector is defined :" << std::endl;
    char frame;
    std::cin >> frame;

    if (frame=='a'){
        operator<<(std::cout, v);

        /*converting from frame b to a */
        Vector2D v1=Tba(v);
        operator<<(std::cout, v1);

        /* converting from frame c to a  */
        Vector2D v2=Tca(v);
        operator<<(std::cout, v2);
        

    }
    else if (frame=='b'){
        operator<<(std::cout, v);

        /* converting from frame a to b  */
        Vector2D v1=Tab(v);
        operator<<(std::cout, v1);

        /* converting from frame c to b */
        Vector2D v2=Tcb(v);
        operator<<(std::cout, v2);


    }
    else{

        operator<<(std::cout, v);

        /* converting from frame a to c  */
        Vector2D v1=Tac(v);
        operator<<(std::cout, v1);

        /*  converting from frame b to c */
        Vector2D v2=Tac(v);
        operator<<(std::cout, v2);

    }


    /************************* reperesenting  Twist 2D in different frames **********************************/

    // std::cout << "Enter twist t :" << std::endl;
    // Twist2D t;
    // operator>>(std::cin, t);


    // std::cout << "Enter frame {a,b or c} in which twist is defined :" << std::endl;
    // char framet;
    // std::cin >> framet;

    // if (framet=='a'){
    //     operator<<(std::cout, t);

    //     /* converting from frame b to a  */
    //     Twist2D t1=t.gettwist_inframe(t,Tba);
    //     operator<<(std::cout, t1);

    //     /*converting from frame c to a  */
    //     Twist2D t2=t.gettwist_inframe(t,Tca);
    //     operator<<(std::cout, t2);

    // }
    // else if (framet=='b'){
    //     operator<<(std::cout, t);

    //     /* converting from frame a to b */
    //     Twist2D t1=t.gettwist_inframe(t,Tab);
    //     operator<<(std::cout, t1);

    //     /* converting from frame c to b  */
    //     Twist2D t2=t.gettwist_inframe(t,Tcb);
    //     operator<<(std::cout, t2);


    // }
    // else{

    //     operator<<(std::cout, t);

    //     /*  converting from frame a to c  */
    //     Twist2D t1=t.gettwist_inframe(t,Tac);
    //     operator<<(std::cout, t1);

    //     /* converting from frame c  b to  */
    //     Twist2D t2=t.gettwist_inframe(t,Tbc);
    //     operator<<(std::cout,t2);

    // }

    return 0;
}