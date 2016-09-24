#include "BugAlgorithms.hpp"
#include <math.h>
#include <iostream>

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
const double PI = 3.14159265359;

BugAlgorithms::BugAlgorithms(Simulator * const simulator) {
    m_simulator = simulator;

    m_mode = STRAIGHT;
    m_hit[0] = m_hit[1] = HUGE_VAL;
    m_leave[0] = m_leave[1] = 0;
    m_distLeaveToGoal = HUGE_VAL;
}

BugAlgorithms::~BugAlgorithms(void) {}

Move BugAlgorithms::Bug0(Sensor sensor) {
    Move move = {0, 0}; //initial movement, double {m_dx, m_dy}

    double goalX  = m_simulator->GetGoalCenterX();  // goal coordinate X
    double goalY  = m_simulator->GetGoalCenterY();  // goal coordinate Y
    double currX  = m_simulator->GetRobotCenterX(); // robot current coordinate X
    double currY  = m_simulator->GetRobotCenterY(); // robot current coordinate Y
    double initX  = m_simulator->GetRobotInitX();
    double initY  = m_simulator->GetRobotInitY();

    double dx_g   = goalX - currX;                  // vector, shows directions by +/- (right/left)
    double dy_g   = goalY - currY;                  // vector, shows directions by +/- (right/left)
    double dx_o   = sensor.m_xmin - currX;
    double dy_o   = sensor.m_ymin - currY;

    double dG     = m_simulator->GetDistanceFromRobotToGoal();
    double dO     = sensor.m_dmin;
    double m      = dy_g/dx_g;
    if(m < 0) m *= -1;

    double theta  = atan2(dy_g, dx_g);
    double theta_o= atan2(dy_o, dx_o);

    double step   = m_simulator->GetStep();
    double limit  = m_simulator->GetWhenToTurn();

    // Just to show where obstacle is at...
    m_hit[0] = sensor.m_xmin;
    m_hit[1] = sensor.m_ymin;

    // TODO Optimize the move, based on (currX, currY) & (goalX, goalY);
    if(dO <= limit) {
        // turn left by 90 degrees
        move.m_dx   = (dy_g / dG) * step * -1;
        move.m_dy   = (dx_g / dG) * step;

        if(dO <= 0.9) {
            // when too close to the wall,
            move.m_dy = (sin(theta_o) / dO) * step * -1;
        }
    } else {
        move.m_dx   = (dx_g / dG) * step; // initial vector drawn to get to the goal
        move.m_dy   = (dy_g / dG) * step; // initial vector drawn to get to the goal
    }

    return move;
}

Move BugAlgorithms::Bug1(Sensor sensor) {
    Move move   = {0, 0}; //initial movement, double {m_dx, m_dy}

    double goalX  = m_simulator->GetGoalCenterX();  // goal coordinate X
    double goalY  = m_simulator->GetGoalCenterY();  // goal coordinate Y
    double currX  = m_simulator->GetRobotCenterX(); // robot current coordinate X
    double currY  = m_simulator->GetRobotCenterY(); // robot current coordinate Y
    double initX  = m_simulator->GetRobotInitX();
    double initY  = m_simulator->GetRobotInitY();

    double dx_g   = goalX - currX;                  // vector, shows directions by +/- (right/left)
    double dy_g   = goalY - currY;                  // vector, shows directions by +/- (right/left)
    double dx_o   = sensor.m_xmin - currX;
    double dy_o   = sensor.m_ymin - currY;

    double dG     = m_simulator->GetDistanceFromRobotToGoal();
    double dO     = sensor.m_dmin;
    double m      = dy_g/dx_g;
    if(m < 0) m *= -1;

    double theta  = atan2(dy_g, dx_g);
    double theta_o= atan2(dy_o, dx_o);

    double step   = m_simulator->GetStep();
    double limit  = m_simulator->GetWhenToTurn();

    // Just to show where obstacle is at...
    m_hit[0] = sensor.m_xmin;
    m_hit[1] = sensor.m_ymin;

    // TODO Optimize the move, based on (currX, currY) & (goalX, goalY);
    if(dO <= limit) {
        // turn left by 90 degrees
        move.m_dx   = (dy_g / dG) * step * -1;
        move.m_dy   = (dx_g / dG) * step;

        if(dO <= 0.6) {
            // when too close to the wall,
            move.m_dy = (sin(theta_o) / dO) * step * -1;
        }
    } else {
        move.m_dx   = (dx_g / dG) * step; // initial vector drawn to get to the goal
        move.m_dy   = (dy_g / dG) * step; // initial vector drawn to get to the goal
    }

    return move;
}

Move BugAlgorithms::Bug2(Sensor sensor) {
    //add your implementation
    Move move ={0,0};

    return move;
}
