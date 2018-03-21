/*#define TESTING true
=======
/*
#define TESTING true
>>>>>>> fed98dfd1dc3743187b16841bef74c9e73bbadf1
#if TESTING

#include <cassert>
#include "flightplan.hpp"
#include <iostream>

int main()
{
    {
        std::cerr << "Verify initalize state is invalid: ";
        flightplan fp;
        assert(!fp.validFlightPlan());

        std::cerr << "Done" << std::endl;
    }

    {
        std::cerr << "Parse # of flightCommands, malformed: ";
        char testPlan[] = "#1";
        flightplan fp;

        fp.parseFlightPlan(testPlan);

        assert(!fp.validFlightPlan());

        std::cerr << "Done" << std::endl;
    }

    {
        std::cerr << "Parse # of flightCommands: ";
        char testPlan[] = "#1;";
        flightplan fp;

        fp.parseFlightPlan(testPlan);

        assert(!fp.validFlightPlan());
        assert(fp.size() == 1);

        std::cerr << "Done" << std::endl;
    }

    {
        std::cerr << "Parse '+': ";
        char testPlan[] = "#1;+0901000;";
        flightplan fp;

        fp.parseFlightPlan(testPlan);

        assert(fp.validFlightPlan());
        assert(fp.size() == 1);
        assert(fp[0].rotationDirection == cw);
        assert(fp[0].heading == 90);
        assert(fp[0].rollTime == 1000);

        std::cerr << "Done" << std::endl;
    }

    {
        std::cerr << "Parse '-': ";
        char testPlan[] = "#1;-0901000;";
        flightplan fp;

        fp.parseFlightPlan(testPlan);

        assert(fp.validFlightPlan());
        assert(fp.size() == 1);
        assert(fp[0].rotationDirection == ccw);
        assert(fp[0].heading == 90);
        assert(fp[0].rollTime == 1000);

        std::cerr << "Done" << std::endl;
    }

    {
        std::cerr << "Parse '~': ";
        char testPlan[] = "#1;~0901000;";
        flightplan fp;

        fp.parseFlightPlan(testPlan);

        assert(fp.validFlightPlan());
        assert(fp.size() == 1);
        assert(fp[0].rotationDirection == shortest);
        assert(fp[0].heading == 90);
        assert(fp[0].rollTime == 1000);

        std::cerr << "Done" << std::endl;
    }

    {
        std::cerr << "Parse multipule, normalize heading: ";
        char testPlan[] = "#2;~0901000;+3601500;";
        flightplan fp;

        fp.parseFlightPlan(testPlan);

        assert(fp.validFlightPlan());
        assert(fp.size() == 2);

        // Test first command.
        assert(fp[0].rotationDirection == shortest);
        assert(fp[0].heading == 90);
        assert(fp[0].rollTime == 1000);

        // Test second command.
        assert(fp[1].rotationDirection == cw);
        assert(fp[1].heading == 0);
        assert(fp[1].rollTime == 1500);

        std::cerr << "Done" << std::endl;
    }

    {
        std::cerr << "Flight- plan hold position: \n";
        char testPlan[] = "#1;+0001000;";
        flightplan fp;

        // Parse phase.
        fp.parseFlightPlan(testPlan);
        assert(fp.validFlightPlan());

        // Rotation test.
        fp.beginRotation(0);
        assert(fp.getTargetAngle(0) == 0);
        assert(fp.getTargetAngle(500) == 0);
        assert(fp.getTargetAngle(1000) == 0);

        std::cerr << "Done" << std::endl;
    }

    {
        std::cerr << "Flight- rotate 90: \n";
        char testPlan[] = "#1;+0901000;";
        flightplan fp;

        // Parse phase.
        fp.parseFlightPlan(testPlan);
        assert(fp.validFlightPlan());

        // Rotation test.
        fp.beginRotation(0);
        assert(fp.getTargetAngle(0) == 0);
        assert(fp.getTargetAngle(500) == 45);
        assert(fp.getTargetAngle(1000) == 90);

        std::cerr << "Done" << std::endl;
    }

    {
        std::cerr << "Flight- keeps state: \n";
        char testPlan[] = "#2;+0901000;+1801000;";
        flightplan fp;

        // Parse phase.
        fp.parseFlightPlan(testPlan);
        assert(fp.validFlightPlan());

        // First command.
        fp.beginRotation(0);
        assert(fp.getTargetAngle(0) == 0);
        assert(fp.getTargetAngle(500) == 45);
        assert(fp.getTargetAngle(1000) == 90);

        // Second command.
        assert(fp.getTargetAngle(1500) == 135);
        assert(fp.getTargetAngle(2000) == 180);

        std::cerr << "Done" << std::endl;
    }

    {
        std::cerr << "Flight- Rolls ccw: \n";
        char testPlan[] = "#2;+0901000;-0001000;";
        flightplan fp;

        // Parse phase.
        fp.parseFlightPlan(testPlan);
        assert(fp.validFlightPlan());

        // First command.
        fp.beginRotation(0);
        assert(fp.getTargetAngle(0) == 0);
        assert(fp.getTargetAngle(500) == 45);
        assert(fp.getTargetAngle(1000) == 90);

        // Second command.
        assert(fp.getTargetAngle(1500) == 45);
        assert(fp.getTargetAngle(2000) == 0);

        std::cerr << "Done" << std::endl;
    }

    {
        std::cerr << "Flight- Rolls shortest: \n";
        char testPlan[] = "#2;~0901000;+1801000;";
        flightplan fp;

        // Parse phase.
        fp.parseFlightPlan(testPlan);
        assert(fp.validFlightPlan());

        // First command.
        fp.beginRotation(0);
        assert(fp.getTargetAngle(0) == 90);
        assert(fp.getTargetAngle(500) == 90);
        assert(fp.getTargetAngle(1000) == 90);

        // Second command.
        assert(fp.getTargetAngle(1500) == 135);
        assert(fp.getTargetAngle(2000) == 180);

        std::cerr << "Done" << std::endl;
    }

    {
        std::cerr << "Flight- Loop test: \n";
        char testPlan[] = "#2;~0901000;+1801000;";
        flightplan fp;

        // Parse phase.
        fp.parseFlightPlan(testPlan);
        assert(fp.validFlightPlan());

        // First command.
        fp.beginRotation(0);
        int angle = 0;
        for (unsigned long l = 0; l < 2500; l += 10)
        {
            angle = fp.getTargetAngle(l);
            std::cerr << angle << std::endl;
            assert((angle >= 0) && (angle < 360));
        }
    }

    return 0;
}

<<<<<<< HEAD
#endif*/
