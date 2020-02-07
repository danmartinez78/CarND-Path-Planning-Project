#ifndef BEHAVIOR_PLANNER_CLASS
#define BEHAVIOR_PLANNER_CLASS

#include "tinyfsm.hpp" // finite state machine library
#include "helpers.h"   // helper functions

struct KEEP_LANE;
struct PLCL;
struct PLCR;
struct LCL;
struct LCR;

struct GO_AROUND : tinyfsm::Event {};

#endif