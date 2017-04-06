#include "Factorial.h""

/*
Your code goes here! See Factorial.h for your method's signature (eg, its args and return value).
*/

int Factorial(int n){
    if(n > 1){
        return n * Factorial(n - 1);
    }
    else return 1;
}