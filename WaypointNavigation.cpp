#include "WaypointNavigation.hpp"
#include <iostream>

WaypointNavigation::WaypointNavigation()
{
  mNavigationState = TARGET_REACHED;
  // Booleans
  aligning    = false;
  targetSet   = false;
  poseSet     = false;
  newWaypoint = false;

  // Ackermann turn parameters
  minTurnRadius    = 0.6; // (in meters)
  maxDisplacementAckermannTurn = 0.25; // (meters from straight line to the next point)
  // Alignment parameters
  maxDisalignment = 10.0 / 180.0 * M_PI;

  // Velocities
  translationalVelocity = 0.05; // [m/s]
  rotationalVelocity = 0.15;    // [rad/s] ... cca 8.6 deg/s
  // Distances
  corridor = .2;
  lookaheadDistance = .6;

}

NavigationState WaypointNavigation::getNavigationState() {
  return mNavigationState;
}

void  WaypointNavigation::setNavigationState(NavigationState state){
  mNavigationState = state;
}


void WaypointNavigation::setPose(base::samples::RigidBodyState& pose)
{
  curPose = pose;
  xr = base::Vector2d(pose.position(0), pose.position(1));
  if (!poseSet && !trajectory.empty()){
    w1 << curPose.position(0), curPose.position(1);
    setSegmentWaypoint(w2, currentSegment);
  }
  poseSet=true;
}

void WaypointNavigation::setLookaheadPoint(base::Waypoint& waypoint)
{
  targetSet = true;
  aligning = false;
  lookaheadPoint = waypoint;
  newWaypoint = true;
}

// Implementation of "pure pursuit" path tracking movement command
void WaypointNavigation::getMovementCommand (base::commands::Motion2D& mc)
{

  //check if std deviation is bigger than the specified std deviation for the current target pose
  for (int i = 0; i<3; i++) {
    //not no sqrt, as both values are sqared, and > is vallid in this case
  if(curPose.cov_position(i,i) > lookaheadPoint.tol_position) {
      std::cout << "Variance of " << i << " is to high " << curPose.cov_position(i,i)
                << " should be smaller than " << lookaheadPoint.tol_position << std::endl;
      mc.translation = 0;
      mc.rotation = 0;
      return;
    }
  }

  if(!targetSet || !poseSet) {
    std::cout << "No target or pose specified" << std::endl;
    mc.translation = 0;
    mc.rotation = 0;
    return;
  }

  std::cout << "X: " << lookaheadPoint.position.x() << " Y: " << lookaheadPoint.position.y() << " Lookahead point, WCF" << std::endl;
  std::cout << "X: " << curPose.position.x() << " Y: " << curPose.position.y() << " Robot position, WCF" << std::endl;

  // Vector to target position
  Eigen::Vector3d driveVector = lookaheadPoint.position - curPose.position;
  driveVector.z() = 0; // Do not care about robot's Z displacement, navigation in 2D plane
  double distToTarget = driveVector.norm();

  // Transform Lookahead point to Robot Coordinate Frame
  Eigen::Vector3d lookaheadPointRCF(0,0,0);
  lookaheadPointRCF = Eigen::AngleAxisd(-curPose.getYaw(), Eigen::Vector3d::UnitZ()) * (lookaheadPoint.position - curPose.position);
  std::cout << "X: " << lookaheadPointRCF.x() << " Y: " << lookaheadPointRCF.y() << " Z: " << lookaheadPointRCF.z() << " Lookahead Point, RCF" << std::endl;;

  std::cout << lookaheadPoint.heading*180/M_PI << " Target Heading in WCF" << std::endl;

  int sign = ( lookaheadPointRCF.x() < 0 ? -1 : 1);
  if( fabs(lookaheadPointRCF.y()) <=  0.001) {                // Straight line motion if Y below [1mm]
    std::cout << "Straight line case" << std::endl;
    mc.translation = sign * translationalVelocity;
    mc.rotation = 0;
  } else {                                      // Turn Required
    double turn_radius = (lookaheadPointRCF.x()*lookaheadPointRCF.x() + lookaheadPointRCF.y()*lookaheadPointRCF.y())
    /(2*lookaheadPointRCF.y());

    double theta = atan2(lookaheadPointRCF.y(),lookaheadPointRCF.x())*2; // In Robot Coordinate Frame
    if (theta > M_PI){
      theta -= 2*M_PI;
    } else if (theta < -M_PI) {
      theta += 2*M_PI;
    }
    double angleToTarget = theta/2;

    std::cout << turn_radius      << " Turn radius" << std::endl;
    std::cout << theta /M_PI*180  << " Theta (deg)" << std::endl;

    // Select Ackermann or Point turn + Straigt line
    // Based on which combination is closer to the target orientation
    // Minimizing the misalignment at target waypoint (presumably points towards the next waypoint)
    double err_straightLine = - (lookaheadPoint.heading - curPose.getYaw());// Target heading in Robot Coordinate frame
    double err_ackermann    = theta +  err_straightLine;               //  By Ackermann turn - Target heading
    err_straightLine       += angleToTarget;                          //   By Straight line  - Target heading

    // Maximum distance from the straight line to the waypoint
    // lies in 1/2 the Ackermann turn angular_velocity
    double distFromLine = distToTarget * (1-cos(angleToTarget));

    // Debug
    std::cout << err_straightLine /M_PI*180  << " Error if by straight line (deg)" << std::endl;
    std::cout << err_ackermann /M_PI*180  << " Error if by Ackermann (deg)" << std::endl;
    std::cout << distFromLine <<  " Distance from straight line (m)" << std::endl;

    // SELECT THE MORE APPROPRIATE MOTION - WHICH ALIGNS THE ROBOT MORE WITH THE TARGET Orientation
    // TODO WEIGHTS CAN BE USED TO INCREASE THE COST OF A POINT TURN
    if( fabs(turn_radius) <= minTurnRadius            ||
      //  fabs(err_straightLine) < fabs(err_ackermann)  ||
        fabs(distFromLine)>= maxDisplacementAckermannTurn )
    {
      std::cout << "Point turn turn case" << std::endl;
      mc.translation = 0;
      mc.rotation = angleToTarget > 0 ? rotationalVelocity : -rotationalVelocity;
    }
    else
    {                                                 // ACKERMANN TURN CASE
      std::cout << "Ackermann turn case" << std::endl;
      // targetRotation    = theta;
      // targetTranslation = turn_radius*targetRotation;
      mc.translation = sign * translationalVelocity;
      mc.rotation    = mc.translation / turn_radius;
    }
  }
  std::cout << mc.translation             << " translational vel. (m/s)" << std::endl;
  std::cout << mc.rotation  / M_PI *180.0 << " rotational vel. (deg/s)" << std::endl;
}

// Implementation of alignment at final destination
void WaypointNavigation::getAlignmentCommand(double &tv, double &rv){
  for (int i = 0; i<3; i++) {
    //not no sqrt, as both values are sqared, and > is vallid in this case
    if(curPose.cov_position(i,i) > lookaheadPoint.tol_position) {
      std::cout << "Variance of " << i << " is to high " << curPose.cov_position(i,i) << " should be smaller than " << lookaheadPoint.tol_position << std::endl;
      tv = 0;
      rv = 0;
      return;
    }
  }
  if(!targetSet || !poseSet) {
    std::cout << "No target or pose specified" << std::endl;
    tv = 0;
    rv = 0;
    return;
  }

  // Align
  Eigen::Vector3d targetPosRobot(0,0,0);
  // (Body <- World)*(World <- Target)        = (Body <- Target)
  // Inverse(World <- Body)*(World <- Target) = (Body <- Target)
  // Get the transformation (Body <- Target)
  Eigen::Quaterniond driveOrientation = curPose.orientation.inverse() * Eigen::Quaterniond(Eigen::AngleAxisd(targetPose.heading, Eigen::Vector3d::UnitZ()));
  // Point in the target forward direction (Virtual target point, but no translation will be done)
  targetPosRobot = driveOrientation * Eigen::Vector3d(1, 0, 0); // Alignment via point turn
  double angleToTarget = atan2(targetPosRobot.y(),targetPosRobot.x()); // In Robot Coordinate Frame
  // Aligns the heading completely so that FWD || FWD, using the smaller-angle point-turn
  tv = 0;
  if ( fabs(angleToTarget) < maxDisalignment ){
    rv = 0 ;
    setNavigationState(TARGET_REACHED);
  } else {
    rv =  angleToTarget > 0 ? 0.05 : -0.05; // TODO speed from config
  }
}


bool WaypointNavigation::testSetNextWaypoint()
{
  if(trajectory.empty())
  {
    std::cout << "Trajectory is empty" << std::endl;
    setNavigationState(TARGET_REACHED);
    return newWaypoint;
  }

  /*
  while(waypointReached(**currentWaypoint)) {
    std::cout << "TARGET REACHED " << std::endl;
    std::vector<base::Waypoint *>::iterator nextWp = currentWaypoint;
    nextWp++;

    if(nextWp != trajectory.end()) // past-the-end element
    {
      setNavigationState(DRIVING);
      newWaypoint = true;
      currentWaypoint++;
      setTargetPose(**currentWaypoint);
    } else { // Target reached, is end of line
      setNavigationState(ALIGNING);
      break;
    }
  }
  */

  //HERE MY CODE FOR UPDATING CURRENT SEGMENT


  bool ret = newWaypoint;
  newWaypoint = false;

  return ret;
}

/*
  Sets the new trajectory and calculates the distances between waypoints
*/
void WaypointNavigation::setTrajectory(std::vector< base::Waypoint *>& t )
{
  // Delete old trajectory
  targetSet = false;
  for(std::vector<base::Waypoint *>::iterator it = trajectory.begin(); it != trajectory.end(); it++)
  {
    delete *it;
  }
  trajectory.clear();
  trajectory = t;

  currentSegment = 0;
  // Add current pose at the begining
  if (poseSet){
    w1 << curPose.position(0), curPose.position(1);
    setSegmentWaypoint(w2, currentSegment);
  }
  
  if(!trajectory.empty()) {
    distanceToNext = new std::vector<double>(trajectory.size()-1);
    base::Vector3d wp;
    for (size_t i = 0; i < distanceToNext->size(); i++) {
      wp  = trajectory.at(i+1)->position; // w2 - w1
      wp -= trajectory.at(i)->position;
      wp.z() = 0;
      distanceToNext->at(i) = wp.norm();
    }
  } else {
    distanceToNext = new std::vector<double>();
  }
  setNavigationState(DRIVING);
}

bool  WaypointNavigation::update(base::commands::Motion2D& mc){
  // 1) Update the current SEGMENT                            //TODO check whether is it the last
  // Select the segment such that robot is not within immediate reach of the 2nd Waypoint
  while ( (w2-xr).norm() <= corridor ){
      if( currentSegment < trajectory.size()-1){
        setSegmentWaypoint(w1, currentSegment);
        setSegmentWaypoint(w2, currentSegment+1);
        currentSegment++;
      } else {
        break;
      }
  }
  std::cout << "Current segment: " << currentSegment << std::endl;

  // 2) Get intersection point with the Path (should also return distance from the segment)
  base::Vector2d xi = getClosestPointOnPath();
  std::cout << "Closest point: (" << xi.x() << ", "
                                  << xi.y() << ") "
  << std::endl;

  // 3) Calculate the distance from the nominal trajectory
  distanceToPath = (xr-xi).norm();
  std::cout << "Distance from nominal: " << distanceToPath << std::endl;


  NavigationState currentState = getNavigationState();
  std::cout << "Nav. state: " << currentState << std::endl;
  // STATEMACHINE - TODO
  switch (currentState) {
    case (NavigationState)DRIVING:
      {
	if ( distanceToPath >= corridor ){
		setNavigationState(OUT_OF_BOUNDARIES);
		mc.translation = 0;
		mc.rotation =0;
		return false;
	}
      double distance;
      distance = distanceToPath + (w2-xi).norm();

      // i) Get the look ahead point segment
      base::Vector2d lineVector, lookaheadPoint2D;
      if (distance > lookaheadDistance) // Lookahead within same seg.
      {
        // ii) Get the look ahead point
        lineVector = w2-w1;
        lineVector.normalize();
        lookaheadPoint2D = xi + lineVector*(lookaheadDistance-distanceToPath);
      }
      else
      { // Find the right segment
        size_t lookaheadSegment;
        lookaheadSegment = currentSegment;
        for (; lookaheadSegment < distanceToNext->size() &&
               distance <= lookaheadDistance;
             lookaheadSegment++){
             distance +=  distanceToNext->at(lookaheadSegment);
        }
        if (distance <= lookaheadDistance){
          // End of trajectory was reached
          setNavigationState(ALIGNING);
          // Will not execute the ALIGNING case within this update?
          mc.translation = 0;
          mc.rotation    = 0;
          return false;
        }
        else
        {
          // ii) Get the look ahead point
          setSegmentWaypoint(l1, lookaheadSegment-1);
          setSegmentWaypoint(l2, lookaheadSegment);
          lineVector = l2-l1;
          lineVector.normalize();
          lookaheadPoint2D = l2 - lineVector*(distance-lookaheadDistance);
        }
      }
      // Set lookahead point: Vector2d -> Waypoint
      lookaheadPoint.position << lookaheadPoint2D(0),lookaheadPoint2D(1),0;
      lookaheadPoint.heading  = atan2(lineVector(1),lineVector(0));
      lookaheadPoint.tol_position = 0.1;
      targetSet = true;
      std::cout << "Lookahead point: (" << lookaheadPoint2D.x() << ", "
                                      << lookaheadPoint2D.y() << ") "
                                      << std::endl;
      // iii) Get motion command to the lookahead point
      getMovementCommand(mc);
      break;
      }
    case ALIGNING:
      {
	std::cout << "Alinging not implemented yet" << std::endl;	
	mc.translation = 0;
	mc.rotation    = 0;
      break;
      }
    case OUT_OF_BOUNDARIES:{
	mc.translation = 0;
	mc.rotation    = 0;
	if ( distanceToPath < corridor ){
	    setNavigationState(DRIVING);
  	}
	break;
    }
    case TARGET_REACHED:

      break;
    default:
      std::cout<<"default case"<<std::endl;
      break;
  }
  return true;
}

base::Vector2d WaypointNavigation::getClosestPointOnPath(){
  // Solving for parameter k such that closestPoint = w1 + k*segVector;
  // 1) segVector = (w2-w1) Vector of the segment line
  // Using the equations
  //    a) w1 + segVector*k = xi
  //    b) xr + perpendicular(segVector)*j = xi
  // This gives [xr - w1] = [perpendicular(segVector), segVector][j; k]
  // 2) Calculate k using the inverse matrix
  // 3) Calculate the point of intersection using a)

  // 1)
  base::Vector2d segVector = w2 - w1;
  segVector.normalize();
  // 2)
  base::Matrix2d inverseL;
  inverseL  << segVector(1), -segVector(0),
               segVector(0),  segVector(1);
  base::Vector2d xi = inverseL*(xr-w1);
  // 3)
  xi = w1 + xi(1)*segVector;
  return xi;
}

bool WaypointNavigation::setSegmentWaypoint(base::Vector2d& waypoint, int indexSegment){
  if (indexSegment > ((int)trajectory.size()-1) || indexSegment<0)
    return false;
  waypoint << trajectory.at(indexSegment)->position(0),
              trajectory.at(indexSegment)->position(1);
  return true;
}

const base::Waypoint* WaypointNavigation::getLookaheadPoint(){
  return &lookaheadPoint;
};

bool WaypointNavigation::waypointReached(base::Waypoint& target) const
{
  //check if we reached target position in respect, to covariance of target position
  for(int i = 0; i< 3; i++){
    double dist = fabs(curPose.position[i] - target.position[i]);
    double reached_dist = target.tol_position;
    std::cout << "Dist " << i << " is " << dist << " reached dist is " << reached_dist << std::endl;
    if(dist > reached_dist)
    return false;
  }

  std::cout << "Waypoint reached " << std::endl;
  return true;
  //return true;
  /*
  Eigen::Quaterniond oriDiffToTarget = curPose.orientation.inverse() * Eigen::Quaterniond(Eigen::AngleAxisd(targetPose.heading, Eigen::Vector3d::UnitZ()));
  Eigen::Vector3d t = oriDiffToTarget * Eigen::Vector3d(1.0,0,0);
  double anglediff = atan2(t.y(), t.x());

  if(fabs(anglediff) < maxDisalignment)
  return true;

  return false;
  */
}
