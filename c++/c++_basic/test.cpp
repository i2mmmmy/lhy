#include <iostream>

void foo(int a, double b)
{
    std::cout << a << std::endl;
    return;
}

void foo(double a, int b)
{
    std::cout << a << std::endl;
    return;
}

int main()
{
    int n = 3; 
    foo(1, 2.2);
    foo(1.1, 2);
    // std::cout << n << std::endl;     
    return 0;
}