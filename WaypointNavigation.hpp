#ifndef WAYPOINT_NAVIGATION_HPP
#define WAYPOINT_NAVIGATION_HPP
#include <Eigen/Geometry>
#include <vector>
#include <base/samples/rigid_body_state.h>
#include <base/waypoint.h>
#include <base/commands/Motion2D.hpp>

namespace waypoint_navigation_lib{

enum NavigationState{
      DRIVING=0,
      ALIGNING,
      TARGET_REACHED,
      OUT_OF_BOUNDARIES,
      NO_TRAJECTORY
};
class WaypointNavigation
{
  public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  	WaypointNavigation();

  	/**
  	* Set positon and orientation, where to drive to
  	*/
  	void setLookaheadPoint(base::Waypoint &pose);

  	/**
  	* Set current orientation and position
  	*/
    bool setPose(base::samples::RigidBodyState &pose);

  	/**
  	* Sets the trajecory the robot should follow
  	*/
  	void setTrajectory(std::vector<base::Waypoint *> &t);

  	/**
  	* Returns the trajectory
  	*/
  	const std::vector<base::Waypoint *> &getTrajectory() const {
  	    return trajectory;
  	}

    	NavigationState getNavigationState();
    	void setNavigationState(NavigationState state);

    	const base::Waypoint* getLookaheadPoint();

	bool update(base::commands::Motion2D& mc);

	bool configure(	double minR,
			double tv, double rv,
			double cr, double lad);
  	/**
  	* Calculates a motion command (Ackermann or Point turn)
  	* given the robot pose and DRIVING mode
  	*/
  	void getMovementCommand (base::commands::Motion2D& mc);

	bool getProgressOnSegment(int segmentNumber,
			 double& progress, double& distAlong, double& distPerpend);
        double getLookaheadDistance();
        void setCurrentSegment(int segmentNumber); // TESTING ONLY - TODO Remove
  private:
	/*
	* MEMBER VARIABLES
	*/
  	NavigationState mNavigationState;
	bool aligning;
  	bool targetSet;
  	bool poseSet;
  	bool newWaypoint;
    bool finalPhase;

  	double minTurnRadius;        	// Minimum turn radius [m]
    double maxDisplacementAckermannTurn;
  	double maxDisalignment;      	// May be used
    double translationalVelocity;
	double rotationalVelocity;
	double corridor; 		// Allowed Distance perpendicular to path segment
	double lookaheadDistance;
	double distanceToPath;
    double targetHeading;

  	base::samples::RigidBodyState curPose;
  	base::Waypoint targetPose;

  	std::vector<double> *distanceToNext;
	std::vector<base::Waypoint *> trajectory;
  	base::Waypoint lookaheadPoint;
    size_t currentSegment;

    base::Vector2d w1, w2, l1, l2, xr;

	/* -------------------------------
	*  PRIVATE FUNCTIONS
	*  -------------------------------*/

	/**
	* Helper function for setting values of Vector2d with X, Y of a trajectory waypoint
	*/
    bool setSegmentWaypoint(base::Vector2d& waypoint, int indexSegment);

	/**
	* Helper function for finding the closes point on the path segment
	* from the current position of the robot
	*/
    base::Vector2d getClosestPointOnPath();
	
	void initilalizeCurrentSegment();

	bool isInsideBoundaries(double& distAlong, double& distPerpend);
};
}
#endif
