/****************************************************************
 *
 * Copyright (c) 2016
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Description: Library for pure-pursuit based path following
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Jan Filip, email:jan.filip@esa.int, jan.filip2@gmail.com
 * Supervised by: Martin Azkarate, email:martin.azkarate@esa.int
 *
 * Date of creation: Dec 2016
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

#ifndef WAYPOINT_NAVIGATION_HPP
#define WAYPOINT_NAVIGATION_HPP
#include <Eigen/Geometry>
#include <vector>
#include <base/samples/rigid_body_state.h>
#include <base/waypoint.h>
#include <base/commands/Motion2D.hpp>

namespace waypoint_navigation_lib{

#define WAYPOINT_NAVIGATION_DEBUG 0

enum NavigationState{
      DRIVING=0,
      ALIGNING,
      TARGET_REACHED,
      OUT_OF_BOUNDARIES,
      NO_TRAJECTORY,
      NO_POSE,
      MOVE_TO_START
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
			   double cr, double lad, bool backward
         );
    
    bool configurePD(double P, double D, double saturation);
    bool configureTol(double TolPos, double TolHeading);
    bool first_waypoint_reached;

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
    bool backwardPerimtted;

  	double minTurnRadius;        	// Minimum turn radius [m]
    double maxDisplacementAckermannTurn;
  	double translationalVelocity;
	double rotationalVelocity;
	double corridor; 		// Allowed Distance perpendicular to path segment
	double lookaheadDistance;
	double distanceToPath;
    double targetHeading;

    // Alignment tolerances
    double defaultTolHeading, defaultTolPos;

    // Alignment controller
    double alignment_deadband, alignment_saturation;
    double headingErr, alignment_P, alignment_D;
    bool pd_initialized;
    base::Time tprev;
    


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
    inline void wrapAngle(double& angle);
    inline void saturation(double& value, double limit);

};
}
#endif
