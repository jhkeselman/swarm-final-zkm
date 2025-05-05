#pragma once
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A data structure to store the position, progress, reward, and current assignment
 * of a food item to a footbot. Is also used in the global data structure
*/
struct SFoodItem {
    // The position of the food item as a vector
    CVector2 Position;
    // The progress level of the item until completion (counts down)
    int Progress;
    // The reward gained from recieving the item
    int Reward;
    // 0 if unassigned task, 1 otherwise
    int Assigned;
};