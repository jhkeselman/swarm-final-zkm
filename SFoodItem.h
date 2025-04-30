#pragma once
/* Definition of the foot-bot motor position sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

struct SFoodItem {
    CVector2 Position;
    int Progress;
    int Reward;
    int Assigned;
};