#include "src/include/geometry/main.hpp"
#include <iostream>
using namespace geo;
int main()
{
    f64 values[36] = {
        1, 2, 3, 4, 5, 6,
        1, 2, 3, 4, 5, 6,
        1, 2, 3, 4, 5, 6,
        1, 2, 3, 4, 5, 6,
        1, 2, 3, 4, 5, 6,
        1, 2, 3, 4, 5, 6
    };

    Matrix m = Matrix(values, 6, 6);
    Matrix m2 = Matrix(values, 6, 6);
    m = m * m2;
    for (int i = 0; i < m.height; i++)
    {
        std::cout<<"[";;
        for (int j = 0; j < m.width; j++)
            std::cout<<" "<<m[i][j];
        std::cout<<"]\n";
    }
}