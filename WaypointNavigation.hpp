#ifndef DUMBTRAJECTORYFOLLOWER_HPP
#define DUMBTRAJECTORYFOLLOWER_HPP
#include <Eigen/Geometry>
#include <vector>
#include <base/samples/rigid_body_state.h>
#include <base/waypoint.h>
#include <base/commands/Motion2D.hpp>

enum NavigationState{
      DRIVING,
      ALIGNING,
      TARGET_REACHED,
      OUT_OF_BOUNDARIES
};


class WaypointNavigation
{
  public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  	WaypointNavigation();

  	/**
  	* set positon and orientation, where to drive to
  	*/
  	void setTargetPose(base::Waypoint &pose);

  	/**
  	* Set current orientation and position
  	*/
  	void setPose(base::samples::RigidBodyState &pose);

  	/**
  	* calculates a translational and rotational velocity
  	* that should drive the robot from the current
  	* pose to the target pose
  	*/
  	void getMovementCommand(double &tv, double &rv);

    /**
  	* calculates a rotational velocity for point turn
  	* that would align the robot with the
  	* orientation of the final target pose
  	*/
  	void getAlignmentCommand(double &tv, double &rv);

  	/**
  	* sets the trajecory the robot should follow
  	*/
  	void setTrajectory(std::vector<base::Waypoint *> &t);

  	/**
  	* tests if the current Waypoint was reached and switches to
  	* the next waypoint in the trajectory
  	*
  	* returns true if new waypoint was selected
  	*/
  	bool testSetNextWaypoint();

  	/**
  	* returns an iterator, that points to the current waypoint in
  	* the trajectory
  	*/
  	std::vector<base::Waypoint *>::const_iterator getCurrentWaypoint() const {
  	    return currentWaypoint;
    }

  	/**
  	* returns the trajectory
  	*/
  	const std::vector<base::Waypoint *> &getTrajectory() const {
  	    return trajectory;
  	}

    NavigationState getNavigationState();
    void setNavigationState(NavigationState state);

    const base::Waypoint* getLookaheadPoint();

    bool update(base::commands::Motion2D& mc);

  private:
  	/*
  	* Calculates if the given waypoint was reached
  	*/
  	bool waypointReached(base::Waypoint &target) const;
    NavigationState mNavigationState;

  	bool newWaypoint;
  	double stopAndTurnAngle;
    double minTurnRadius;
  	double tvP;
  	double rvP;
  	double maxDisalignment;
    double maxDisplacementAckermannTurn;
  	double reachedHysteresisRatio;
    double targetTranslation;
    double targetRotation;
    bool aligning;
  	bool targetSet;
  	bool poseSet;
  	base::samples::RigidBodyState curPose;
  	base::Waypoint targetPose;


  	std::vector<double> *distanceToNext;
    std::vector<base::Waypoint *> trajectory;
  	std::vector<base::Waypoint *>::iterator currentWaypoint;
    base::Waypoint lookaheadPoint;
    size_t currentSegment;
    double corridorSq; // Allowed Distance perpendicular to path segment
    double lookaheadDistance;
    base::Vector2d w1, w2, xr, l1, l2;

    bool setSegmentWaypoint(base::Vector2d& waypoint, int indexSegment);
    base::Vector2d getClosestPointOnPath();

};

#endif
