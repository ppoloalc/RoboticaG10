#include <algorithm>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <map>
#include <random>

#include <Eigen/Dense>


//Si hay & no realiza la copia
void funcion(std::vector<int> &v)
{
    //Copia el vector al espacio de la funcion
    auto a = v;
}

int main()
{

    std::random_device rd;
    std::mt19937 gen(rd());
    /*std::uniform_int_distribution<int> dice(1, 6);
    std::uniform_real_distribution<double> percent(0.0, 100.0);
    std::normal_distribution<double> height(170.0, 10.0);
    // Height: mean=170cm, std=10cm
    std::cout << "=== Random Examples ===\n";
    // Generate some dice rolls
    std::cout << "Dice rolls: ";
    for (int i = 0; i < 5; ++i)
    { std::cout <<  dice(gen) << " ";
        std::cout << percent(gen);
        std::cout << height(gen);
    }
    std::cout << "\n"; */

//Punto 4.a
//     auto start = std::chrono::high_resolution_clock::now();
//
//     auto sort = [](std::tuple<std::string, int> t1, std::tuple<std::string, int> t2)
//     {
//
//         int a = std::get<1>(t1);
//         int b = std::get<1>(t2);
//
//         return a<b;
//     };
//
//     std::vector<std::tuple<std::string, int>> v;
//     v.push_back(std::make_tuple("0", 0));
//     v.push_back(std::make_tuple("5", 5));
//     v.push_back(std::make_tuple("4", 4));
//     v.push_back(std::make_tuple("2", 2));
//     v.push_back(std::make_tuple("1", 1));
//     v.push_back(std::make_tuple("3", 3));
//
//     std::sort(v.begin(), v.end(), sort);
//
//     for ( int i = 0; i < v.size(); i++)
//     {
//         printf("%s %i\n", std::get<0>(v[i]).c_str(), std::get<1>(v[i]));
//     }
//
// //Punto 4.b
//
//     auto degstorads = [](int degrees)
//     {
//         return (degrees*(std::numbers::pi))/180.0;
//     };
//
//     int d = 90;
//     printf("Degrees: %i\n", d);
//     printf("Radians: %f\n", degstorads(d));
//
// // Punto 5
//     int a = 5;
//     int&& r = std::move(a);
//
// //Punto 7
//
//     auto end = std::chrono::high_resolution_clock::now();
//     const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //printf("%d", ms);


//Punto 8

    //std::thread t3([](int n) {std::cout << n << std::endl;},3);

//Punto 11




    std::vector<int> vect(1000000000);
    vect.push_back(1);
    auto inicio = std::chrono::high_resolution_clock::now();
    //funcion(vect);
    //min_element(vect.begin(), vect.end());
    std::sort(vect.begin(), vect.end());

    auto fin = std::chrono::high_resolution_clock::now();
    const auto tiempo = std::chrono::duration_cast<std::chrono::milliseconds>(fin - inicio);
    printf("%d", tiempo);

    //Punto 16
    struct A
    {  float init = 0, end = 0, step = 0;  };
    auto B = A{.init=1, .end=2, .step=3};

    //Punto 17
    struct Node{ int id; std::vector<int> links;};
    std::map<int, Node> graph;
    for(int i=0; i<100; i++)
        graph.emplace(std::make_pair(i, Node{i, std::vector<int>()}));



    for(auto &[key, value] : graph)
    {
        std::uniform_int_distribution<int> vecino(0, 5);
        std::uniform_int_distribution<int> links(0, 99);

        int vecinos = vecino(gen);
        for(int j=0; j<vecinos; j++)
            value.links.emplace_back( links(gen));
    }

    //Punto 19
    struct Object
    {
        int id;
        uint timestamp;
    };
    std::map<int, Object> semantic_map;

    //Se insertan en semantic_map 2 tuplas entero y objeto
    semantic_map.insert(std::make_pair(0,Object{0, 45}));
    semantic_map.insert(std::make_pair(1,Object{1, 3}));

    //Lambda que devuelve true si el tiempo del objeto del mapa es mayor que 5
   // auto removable = semantic_map | std::views::values | std::views::filter([p = params](auto o) { return o.timestamp > 5; });

    //Elimina los elementos cuyo timestamp es mayor que 5
    //for (const auto r: removable)
        //semantic_map.erase(r.id);


return 0;
}
