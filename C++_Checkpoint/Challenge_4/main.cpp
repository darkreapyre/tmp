/*
Class Basics

In this challenge, I want to flip the tables a bit.
In the previous quizzes, I've been giving you header files and asking you to implement methods from them.
In this quiz, I'm going to give you a .cpp file (Car.cpp) and ask you to write the corresponding header file.

In Car.cpp, you'll find the implementation of a simple Car class.
This is a very unreliable car that has a 50/50 chance of being broken after every drive.

I want you to examine Car.cpp and write the corresponding header file, Car.h.
The code you'll find below won't compile without a working header file.
Check out the compiler errors and make it work!

Here is a test of your code. Feel free to play with it but there's
no need to edit this file. Remember, you're only trying to make your code
compile.
*/
#include "Car.h"

int main()
{
    Car car;
    
    // try to drive 10 times
    for (int i = 0; i < 10; i++) {
        bool didDrive = car.drive();
        if (!didDrive) {
            // car is broken! must fix it
            car.fix();
        }
    }
    
    return 0;
}
