//
// Created by Andrew on 2024/8/27.
//

#ifndef FINALEXAM_FRAMEWORK_HEADFILE_H
#define FINALEXAM_FRAMEWORK_HEADFILE_H

#include "DebugC.h"
#include "pidC.hpp"
#include "filter.hpp"
#include "kalman.hpp"
#include "Sliding.h"
#include "packet.hpp"



class algorithm
{
public:
    cPID c_PID;
    cSMC c_SMC;
};

#endif //FINALEXAM_FRAMEWORK_HEADFILE_H
