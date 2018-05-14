/*
//
// Created by alex on 4/29/18.
//
#include <iostream>
#include <cassert>
#include "bangbang.h"

int main()
{
    // The old flight system.
    {
        int result = frame180(180, 0);
        std::cerr << result << std::endl;
        assert(result == 0);
    }
    // 180 degree test.
    {
        int result = frame180(0, 180);
        std::cerr << result << std::endl;
        assert(result == 0);
    }
    // 90 degree test.
    {
        int result = frame180(90, 30);
        std::cerr << result << std::endl;
        assert(result == 120);
    }
    // 270 degree test.
    {
        int result = frame180(270, 30);
        std::cerr << result << std::endl;
        assert(result == 300);
    }
    return 0;
}
*/
