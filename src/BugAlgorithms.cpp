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

/**
 * Personal Reference
 * 1) ~FunctionName implies that it is a destructor
 * 2) :: a.k.a 'class resolution operator'
 * 3) (2) means that it is declaringa a member object
 */

const double PI = 3.14159265359;

// Just a constructor
// Simulator loads up the obstacle.txt to generate the simulation env.
BugAlgorithms::BugAlgorithms(Simulator * const simulator) {
  m_simulator = simulator;   

	m_mode = STRAIGHT;
	m_hit[0] = m_hit[1] = HUGE_VAL;
	m_leave[0] = m_leave[1] = 0;
	m_distLeaveToGoal = HUGE_VAL;
}

BugAlgorithms::~BugAlgorithms(void) {
    // do not delete m_simulator  
}

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
	double dx_o   = sensor.m_xmin - currX;          // vector, shows directions by +/- (right/left)
	double dy_o   = sensor.m_ymin - currY;          // vector, shows directiosn by +/- (right/left)
  
  double rad    = atan2(dy_o, dx_o);

  if(sensor.m_dmin >= m_simulator->GetWhenToTurn()) {
    // to goal
    move.m_dx   = m_simulator->GetStep() * (dx_g / m_simulator->GetDistanceFromRobotToGoal());
    move.m_dy   = m_simulator->GetStep() * (dy_g / m_simulator->GetDistanceFromRobotToGoal());
  } else {
    move.m_dx   = cos(rad);
    move.m_dy   = sin(rad);
  }
  return move;
}

Move BugAlgorithms::Bug1(Sensor sensor) {
    Move move 	= {0, 0}; //initial movement, double {m_dx, m_dy}
	
	double gX 	= m_simulator->GetGoalCenterX(); 
	double gY 	= m_simulator->GetGoalCenterY();
	double rX 	= m_simulator->GetRobotCenterX();
	double rY 	= m_simulator->GetRobotCenterY();
	double oX	= sensor.m_xmin;
	double oY	= sensor.m_ymin;

	double dgX	= gX - rX;
	double dgY  = gY - rY;
	double doX  = oX - rX;
	double doY  = oY - rY;
	double dG   = sqrt(pow(dgX, 2) + pow(dgY, 2));
	double dO   = sqrt(pow(doX, 2) + pow(doY, 2));

	double mX   = dgX / dG;
	double mY   = dgY / dG;

	double rX_n = rX + mX;
	double rY_n = rY + mY;
	double doX_n= oX - rX_n;
	double doY_n= oY - rY_n;
	double dO_n = sqrt(pow(doX_n, 2) + pow(doY_n, 2));

	if(dO_n <= 1) {
		mX = doX_n / dO_n;
		mY = doY_n / dO_n;
		move.m_dx = mX * 0.06;
		move.m_dy = mY * 0.06;
		printf("obs || dO: %f\n", dO);
	} else {
		move.m_dx = mX * 0.06;
		move.m_dy = mY * 0.06;
		printf("goal\n");
	}

    return move;
}

Move BugAlgorithms::Bug2(Sensor sensor) {
    //add your implementation
    Move move ={0,0};
    
    return move;
}
