#ifndef TIMER_H
#define TIMER_H
#include <thread>
#include <chrono>
#include <functional>
#include <future>
#include <cstdio>
#include <iostream>

//Concept to ensure the parameter is callable with no arguments
template<typename T>
//Concept es para restringir lo que se puede hacer con el template a la hora de llamarlo en tiempo de compilacion
concept Callable = std::invocable<T>;

class Timer
{
   public:
       Timer(){};
       template <class T>
       //connect es template porque al llamarlo puedes asignarle a f el tipo que quieras
       void connect(T f) //f tiene las restricciones del template T
       {
           std::thread([this, f = std::move(f)]()
           {
               while(true)
               {
                     if(go.load())
                         std::invoke(f); //Equivalente a f();
                     std::this_thread::sleep_for(std::chrono::milliseconds(period.load()));
               }
           }).detach();
           //Detach hace que el hilo no deje de ejecutarse al llegar a la llave
       };
       void start(int p)
       {
            // COMPLETAR
           period.store(p);
           go.store(true);
       };
        void stop()
       {
	        // COMPLETAR
            go.store(false);
       };
       void setInterval(int p)
      {
   	        // COMPLETAR
            period.store(p);
      }
   private:
       //El atomic evita que los hilos accedan a la vez a la misma variable
       std::atomic_bool go = false;
       std::atomic_int period = 0;
};
#endif // TIMER_H