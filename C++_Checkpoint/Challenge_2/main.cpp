/* Control Flow

In this quiz, I want you to write a for loop.
Your loop will live inside a function that will be passed an int, n, and a string, str.
Your loop should print the str n times.

There's another twist here - you will not need to touch main.cpp this time.
Instead, your code will be written inside the eponymous function in PrintString.cpp.
Make note of the way that main.cpp #includes PrintString.h.
You'll be #includeing files soon.
No need to change this file (unless you want to change the test case below).
*/

#include <iostream>
#include "PrintString.h"

using namespace std;

int main()
{
    PrintString("This is a test.", 10);
}
