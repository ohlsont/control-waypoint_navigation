#ifndef DUMBTRAJECTORYFOLLOWER_HPP
#define DUMBTRAJECTORYFOLLOWER_HPP
#include <Eigen/Geometry>
#include <vector>
#include <base/samples/rigid_body_state.h>
#include <base/waypoint.h>
#include <base/commands/Motion2D.hpp>

enum NavigationState{
      DRIVING=0,
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
  	void setLookaheadPoint(base::Waypoint &pose);

  	/**
  	* Set current orientation and position
  	*/
        bool setPose(base::samples::RigidBodyState &pose);

  	/**
  	* calculates a translational and rotational velocity
  	* that should drive the robot from the current
  	* pose to the target pose
  	*/
  	void getMovementCommand (base::commands::Motion2D& mc);

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

    bool aligning;
  	bool targetSet;
  	bool poseSet;
  	bool newWaypoint;

  	double minTurnRadius;        // Minimum turn radius [m]
    double maxDisplacementAckermannTurn;
  	double maxDisalignment;      // May be used
    double translationalVelocity;
    double rotationalVelocity;
    double corridor; // Allowed Distance perpendicular to path segment
    double lookaheadDistance;
    double distanceToPath;

  	base::samples::RigidBodyState curPose;
  	base::Waypoint targetPose;

  	std::vector<double> *distanceToNext;
    std::vector<base::Waypoint *> trajectory;
  	base::Waypoint lookaheadPoint;
    size_t currentSegment;

    base::Vector2d w1, w2, l1, l2, xr;

    bool setSegmentWaypoint(base::Vector2d& waypoint, int indexSegment);
    base::Vector2d getClosestPointOnPath();

};

#endif
