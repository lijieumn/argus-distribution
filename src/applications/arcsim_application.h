//
// Created by lijie on 9/28/18.
//

#ifndef ARGUS_DISTRIBUTION_ARCSIM_APPLICATION_H
#define ARGUS_DISTRIBUTION_ARCSIM_APPLICATION_H

#include "application.h"
namespace applications {

    class ArcsimApplication : public Application {
    private:

    public:
        ArcsimApplication(int argc, char const *argv[]);

        void init();
    };

}

#endif //ARGUS_DISTRIBUTION_ARCSIM_APPLICATION_H
