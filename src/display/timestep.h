//
// Created by lijie on 9/27/18.
//

#ifndef ARGUS_DISTRIBUTION_TIMESTEP_H
#define ARGUS_DISTRIBUTION_TIMESTEP_H

#include "drawable.h"

namespace display {
    class Timestep {
    protected:
        Drawable clothDrawable;
        Drawable obstacleDrawable;
    public:
        virtual void step() = 0;
        virtual void resetDrawable() = 0;
        virtual void saveScreenshot(int width, int height) = 0;
        Drawable& getClothDrawable();
        Drawable& getObstacleDrawable();
    };
} // display

#endif //ARGUS_DISTRIBUTION_TIMESTEP_H
