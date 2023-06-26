#include <limits>
#include <iostream>

int main()
{
    double f = std::numeric_limits<double>::quiet_NaN();
    std::cout<<(f==f);
}