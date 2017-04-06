/*
In this quiz, I want you to write a function called factorial that returns an int.
A factorial is a mathematical operation designated by a ! operator and returns the
product of a number and every whole number between it and 0. For example:

4!=4⋅3⋅2⋅1=24

You'll write your code within Factorial.cpp
*/

#include <iostream>
#include "Factorial.h"

int main()
{
    // feel free to change this test case!
    int value = Factorial(6);
    std::cout << "6! should equal 720. Your Factorial method returned:" << std::endl;
    std::cout << value << std::endl;
}
