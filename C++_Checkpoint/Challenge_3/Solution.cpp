/*
Here's my solution:

#include <Factorial.h>

int Factorial(int n)
{
    int result = 1;

    for (int i = n; i > 0; i--) {
        result *= i;
    }

    return result;
}

I took advantage of the *= operator to multiply result by a lower int and then reset itself to the new value simultaneously.
*/