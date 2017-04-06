/*Here's my solution:

// Doubler.h

void Doubler(int& n);

// Doubler.cpp

#include <Doubler.h>

void Doubler(int& n)
{
    n *= 2;
}

The tricky part here was making sure to use the & operator to pass n as a reference.
*/