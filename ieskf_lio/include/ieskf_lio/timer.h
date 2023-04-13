#pragma once

#ifndef TIMER_H
#define TIMER_H

#include <ctime>
#include <chrono>
#include <stdio.h>
#include <iostream>

class Timer
{
public:
    Timer(const char *nameIn)
    {
        name = nameIn;
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> dt = end - start;
        return dt.count() * 1000;
    }

    void tic_toc()
    {
        std::cout << "The process " << name << " takes " << toc() << " ms." << std::endl;
    }

private:
    const char *name;
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

#endif // TIMER_H
