#include <flex.h>
#include <stddef.h>
#include <iostream>

const int maxParticles = 65536;
const int maxDiffuse = 0;

int main(void)
{
        flexInit();
        FlexSolver* solver = flexCreateSolver(maxParticles, maxDiffuse);
        int finished = 0;
        FlexTimers timer = FlexTimers();

        while (finished < 24*150)
        {
                const float dt = 1.0f/24.0f;

                // tick solver with one sub-step
                flexUpdateSolver(solver, dt, 1, &timer);
                finished ++;
        }

        std::cout << timer.mCollideParticles << std::endl;
        flexDestroySolver(solver);
        flexShutdown();

        return 0;
}