#include <algorithm>
#include <iostream>
#include <vector>
// TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or click the <icon src="AllIcons.Actions.Execute"/> icon in the gutter.
int main()
{
//Punto 4.a

    auto sort = [](std::tuple<std::string, int> t1, std::tuple<std::string, int> t2)
    {

        int a = std::get<1>(t1);
        int b = std::get<1>(t2);

        return a<b;
    };

    std::vector<std::tuple<std::string, int>> v;
    v.push_back(std::make_tuple("0", 0));
    v.push_back(std::make_tuple("5", 5));
    v.push_back(std::make_tuple("4", 4));
    v.push_back(std::make_tuple("2", 2));
    v.push_back(std::make_tuple("1", 1));
    v.push_back(std::make_tuple("3", 3));

    std::sort(v.begin(), v.end(), sort);

    for ( int i = 0; i < v.size(); i++)
    {
        printf("%s %i\n", std::get<0>(v[i]).c_str(), std::get<1>(v[i]));
    }

//Punto 4.b

    auto degstorads = [](int degrees)
    {
        return (degrees*(std::numbers::pi))/180.0;
    };

    int d = 90;
    printf("Degrees: %i\n", d);
    printf("Radians: %f\n", degstorads(d));

return 0;
}