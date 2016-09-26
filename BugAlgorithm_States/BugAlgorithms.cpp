#include "BugAlgorithms.hpp"
#include <iostream>
#include <math.h>


BugAlgorithms::BugAlgorithms(Simulator * const simulator)
{
    m_simulator = simulator;   
    m_mode = STRAIGHT;
    m_hit[0] = m_hit[1] = HUGE_VAL;
    m_leave[0] = m_leave[1] = 0;
    m_distLeaveToGoal = HUGE_VAL;  
}

BugAlgorithms::~BugAlgorithms(void)
{
    //do not delete m_simulator  
}

Move BugAlgorithms::Bug0(Sensor sensor)
{
	//std::cout << "\nx!!!!";
	Move move = {0, 0};
	switch (m_mode)
	{		
		case STRAIGHT: 	// Move in a straight line towards the goal
		{
			//Check for possible collision with boundary
			if (sensor.m_dmin <= m_simulator->GetWhenToTurn())
			{
				m_mode = AROUND_AND_AWAY_FROM_HIT_POINT;
			}
			else // Just keep going straight
			{
				double stepsToGoal = m_simulator->GetDistanceFromRobotToGoal() / m_simulator->GetStep();
				move.m_dx = (m_simulator->GetGoalCenterX() - m_simulator->GetRobotCenterX()) / stepsToGoal;
				move.m_dy = (m_simulator->GetGoalCenterY() - m_simulator->GetRobotCenterY()) / stepsToGoal;
			}		
			break;
		}
		case AROUND_AND_AWAY_FROM_HIT_POINT:	// Save hit point (x,y) and change state
		{
			m_hit[0] = m_simulator->GetRobotCenterX();
			m_hit[1] = m_simulator->GetRobotCenterY();
			m_mode = AROUND;
			break;
		}
		case STRAIGHT_AND_AWAY_FROM_LEAVE_POINT: // Save leave point (x,y) and change state
		{
			m_leave[0] = m_simulator->GetRobotCenterX();
			m_leave[1] = m_simulator->GetRobotCenterY();
			m_mode = STRAIGHT;
		}
		case AROUND_AND_TOWARD_LEAVE_POINT:
		{
			break;
		}
		case AROUND: // Now the robot follows boundaries
		{
			/*std::cout << "POSX ROBOT: ";
			std::cout <<  m_simulator->GetRobotCenterX();
			std::cout << "\nPOSY ROBOT: ";
			std::cout << m_simulator->GetRobotCenterY();
			std::cout << "\n";*/
			
			// Move towards the next closest boundary point
			//move.m_dx = sensor.m_xmin - m_simulator->GetRobotCenterX();
			//move.m_dy = sensor.m_ymin - m_simulator->GetRobotCenterY();
	//	move.m_dx = (sensor.m_xmin + m_simulator->GetWhenToTurn() + 0.006) - m_simulator->GetRobotCenterX();
		//	move.m_dy = (sensor.m_ymin + m_simulator->GetWhenToTurn() + 0.006) - m_simulator->GetRobotCenterY();
			
			// Dx and Dy that make up the step distance (0.006)
			move.m_dx = -0.042; 
			move.m_dy = -0.042;
			m_mode = STRAIGHT;
			break;
		}
	}
	return move;
}

Move BugAlgorithms::Bug1(Sensor sensor) // TODO NEED TO DEFINE MOVEMENT AFTER HITPOINT/BEFORE STRAIGHT MODE
{
	Move move = { 0, 0 };
	switch (m_mode)
	{
		case STRAIGHT: 	// Move in a straight line towards the goal
		{
			//Check for possible collision with boundary
			if (sensor.m_dmin <= m_simulator->GetWhenToTurn())
			{
				m_mode = AROUND_AND_AWAY_FROM_HIT_POINT;
			}
			else // Just keep going straight
			{
				double stepsToGoal = m_simulator->GetDistanceFromRobotToGoal() / m_simulator->GetStep();
				move.m_dx = (m_simulator->GetGoalCenterX() - m_simulator->GetRobotCenterX()) / stepsToGoal;
				move.m_dy = (m_simulator->GetGoalCenterY() - m_simulator->GetRobotCenterY()) / stepsToGoal;
			}
			break;
		}
		case AROUND_AND_AWAY_FROM_HIT_POINT:	// Save hit point (x,y) and change state
		{
			m_hit[0] = m_simulator->GetRobotCenterX();
			m_hit[1] = m_simulator->GetRobotCenterY();
			m_mode = AROUND;
			break;
		}
		case STRAIGHT_AND_AWAY_FROM_LEAVE_POINT: // Save leave point (x,y) and change state
		{
			m_leave[0] = m_simulator->GetRobotCenterX();
			m_leave[1] = m_simulator->GetRobotCenterY();
			m_mode = STRAIGHT;
		}
		case AROUND_AND_TOWARD_LEAVE_POINT:
		{
			break;
		}
		case AROUND: // Now the robot follows boundaries
		{
			/*std::cout << "POSX ROBOT: ";
			std::cout <<  m_simulator->GetRobotCenterX();
			std::cout << "\nPOSY ROBOT: ";
			std::cout << m_simulator->GetRobotCenterY();
			std::cout << "\n";*/

			// Move towards the next closest boundary point
			//move.m_dx = sensor.m_xmin - m_simulator->GetRobotCenterX();
			//move.m_dy = sensor.m_ymin - m_simulator->GetRobotCenterY();

			// Move towards the next closest boundary point
			move.m_dx = (sensor.m_xmin + m_simulator->GetWhenToTurn()) - m_simulator->GetRobotCenterX();
			move.m_dy = (sensor.m_ymin + m_simulator->GetWhenToTurn()) - m_simulator->GetRobotCenterY();

			// Dx and Dy that make up the step distance (0.006)
			//move.m_dx = 0;
		//	move.m_dy = 0;
			m_mode = STRAIGHT;
			break;
		}
	}
	return move;
}

Move BugAlgorithms::Bug2(Sensor sensor)
{
	Move move = { 0, 0 };
	switch (m_mode)
	{
		case STRAIGHT: 	// Move in a straight line towards the goal
		{	
			//Check for possible collision with boundary
			if (sensor.m_dmin <= m_simulator->GetWhenToTurn())
			{
				m_mode = AROUND_AND_AWAY_FROM_HIT_POINT;
			}
			else // Just keep going straight
			{
				double stepsToGoal = m_simulator->GetDistanceFromRobotToGoal() / m_simulator->GetStep();
				move.m_dx = (m_simulator->GetGoalCenterX() - m_simulator->GetRobotCenterX()) / stepsToGoal;
				move.m_dy = (m_simulator->GetGoalCenterY() - m_simulator->GetRobotCenterY()) / stepsToGoal;
			}
			break;
		}
		case AROUND_AND_AWAY_FROM_HIT_POINT:	// Save hit point (x,y) and change state
		{
			m_hit[0] = m_simulator->GetRobotCenterX();
			m_hit[1] = m_simulator->GetRobotCenterY();
			m_mode = AROUND;
			break;
		}
		case STRAIGHT_AND_AWAY_FROM_LEAVE_POINT: // Save leave point (x,y) and change state
		{
			m_leave[0] = m_simulator->GetRobotCenterX();
			m_leave[1] = m_simulator->GetRobotCenterY();
			m_mode = STRAIGHT;
		}
		case AROUND_AND_TOWARD_LEAVE_POINT:
		{
			m_mode = STRAIGHT_AND_AWAY_FROM_LEAVE_POINT;
			break;
		}
		case AROUND: // Now the robot follows boundaries
		{
			/*std::cout << "POSX ROBOT: ";
			std::cout <<  m_simulator->GetRobotCenterX();
			std::cout << "\nPOSY ROBOT: ";
			std::cout << m_simulator->GetRobotCenterY();
			std::cout << "\n";*/

			// Move towards the next closest boundary point
			//move.m_dx = sensor.m_xmin - m_simulator->GetRobotCenterX();
			//move.m_dy = sensor.m_ymin - m_simulator->GetRobotCenterY();

			// Move towards the next closest boundary point
			move.m_dx = (sensor.m_xmin + m_simulator->GetWhenToTurn()) - m_simulator->GetRobotCenterX();
			move.m_dy = (sensor.m_ymin + m_simulator->GetWhenToTurn()) - m_simulator->GetRobotCenterY();

			// Dx and Dy that make up the step distance (0.006)
			//move.m_dx = 0;
			//	move.m_dy = 0;

			// Once the boundary following is near the straight line from origin to goal, we change mode
			m_mode = STRAIGHT;
			if(m_simulator->IsPointNearLine(m_simulator->GetRobotCenterX(), m_simulator->GetRobotCenterY(), m_simulator->GetRobotInitX(), 
				m_simulator->GetRobotInitY(), m_simulator->GetGoalCenterX(), m_simulator->GetGoalCenterY()))
			{
				m_mode = STRAIGHT_AND_AWAY_FROM_LEAVE_POINT;
			}
			
			break;
		}
	}
	return move;
}

       


