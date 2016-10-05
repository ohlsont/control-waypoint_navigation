#include "WaypointNavigation.hpp"
#include <iostream>

WaypointNavigation::WaypointNavigation()
{
  mNavigationState = TARGET_REACHED;
  stopAndTurnAngle = 30.0 / 180.0 * M_PI;
  minTurnRadius    = 0.6; // (in meters)
  tvP = 1.0;
  rvP = 1.0;
  maxDisalignment = 10.0 / 180.0 * M_PI;
  maxDisplacementAckermannTurn = 0.25; // (meters from straight line to the next point)
  targetTranslation = 0.0;
  targetRotation    = 0.0;
  aligning    = false;
  targetSet   = true;
  poseSet     = true;
  reachedHysteresisRatio = 2.0;
  newWaypoint = false;
}

NavigationState WaypointNavigation::getNavigationState() {
  return mNavigationState;
}

void  WaypointNavigation::setNavigationState(NavigationState state){
  mNavigationState = state;
}


void WaypointNavigation::setPose(base::samples::RigidBodyState& pose)
{
  //std::cout << "DTF: Got new pose " << pose.position;
  poseSet = true;
  curPose = pose;
}

void WaypointNavigation::setTargetPose(base::Waypoint& pose)
{
  //std::cout << "DTF: Got new target pose " << pose.position;
  targetSet = true;
  aligning = false;
  targetPose = pose;
  newWaypoint = true;
}

// Implementation of "pure pursuit" path tracking movement command
void WaypointNavigation::getMovementCommand ( double& tv, double& rv )
{

  //check if std deviation is bigger than the specified std deviation for the current target pose
  for (int i = 0; i<3; i++) {
    //not no sqrt, as both values are sqared, and > is vallid in this case
    if(curPose.cov_position(i,i) > targetPose.tol_position) {
      std::cout << "Variance of " << i << " is to high " << curPose.cov_position(i,i) << " should be smaller than " << targetPose.tol_position << std::endl;
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

  std::cout << "X: " << targetPose.position.x() << " Y: " << targetPose.position.y() << " Z: " << targetPose.position.z() << " Target Position in World Coordinates" << std::endl;
  std::cout << "X: " << curPose.position.x() << " Y: " << curPose.position.y() << " Z: " << curPose.position.z() << " Own Position in World Coordinates" << std::endl;
  //calculate vector to target position
  Eigen::Vector3d driveVector = targetPose.position - curPose.position;
  driveVector(2) = 0; // Do not care about robot's Z displacement, navigation in 2D plane
  double distToTarget = driveVector.norm();

  Eigen::Vector3d targetPosRobot(0,0,0);
  targetPosRobot = curPose.orientation.inverse() * (targetPose.position - curPose.position);
  std::cout << "X: " << targetPosRobot.x() << " Y: " << targetPosRobot.y() << " Z: " << targetPosRobot.z() << " Target Position in Robot Coordinates" << std::endl;;
  /*/ DEBUG PRINT OF ORIENTATIONS
  std::cout << curPose.getYaw()*180/M_PI  << " Robot Yaw" << std::endl;        // RBS - orientation
  std::cout << targetPose.heading*180/M_PI << "Target Heading" << std::endl;  // Waypoint, has heading in WCF
  /**/

  // NO MOTION REQUIRED
  if( targetPosRobot.y() == 0 && targetPosRobot.x() == 0){
    // TODO rateher within position tollerance radius
    std::cout << "No motion required, TODO: all align if final" << std::endl;
    tv =0; rv = 0;
    return;
  }

  std::cout << targetPose.heading*180/M_PI << " Target Heading in WCF" << std::endl;  // Waypoint, has heading in WCF
  double timeToGo;
  //calulate angle to target position
  int sign = ( targetPosRobot.x() < 0 ? -1 : 1);
  if( fabs(targetPosRobot.y()) <=  0.001) {                // Straight line motion if Y below [1mm]
    std::cout << "Straight line case" << std::endl;
    targetTranslation = distToTarget*sign;
    tv = 0.05 * sign;                             // TODO add forward speed to Config
    targetRotation = 0;
    rv = 0.0;
    timeToGo = targetTranslation/tv;
  } else {                                      // Turn Required
    double turn_radius = (targetPosRobot.x()*targetPosRobot.x() + targetPosRobot.y()*targetPosRobot.y())
    /(2*targetPosRobot.y());
    //double theta = atan(targetPosRobot.x()/ fabs(turn_radius-targetPosRobot.y()) )*( targetPosRobot.y() > 0 ? 1 : -1);
    double theta = atan2(targetPosRobot.y(),targetPosRobot.x())*2; // In Robot Coordinate Frame
    if (theta > M_PI){
      theta -= 2*M_PI;
    } else if (theta < -M_PI) {
      theta += 2*M_PI;
    }
    double angleToTarget = theta/2;

    std::cout << turn_radius      << " Turn radius" << std::endl;
    std::cout << theta /M_PI*180  << " Theta (deg)" << std::endl;

    // Select Ackermann or Point turn + Straigt line
    // Based on which combo is closer to the target orientation
    // Minimizing the misalignment at target waypoint (presumably points towards the next waypoint)
    double err_straightLine = - (targetPose.heading - curPose.getYaw());// Target heding in Robot Coordinate frame
    double err_ackermann    = theta +  err_straightLine;               // Target - Angle of arival by Ackermann turn
    err_straightLine       += angleToTarget;                          // Target - By Straight line

    // Maximum distance from the straight line to the waypoint
    // lies in 1/2 the Ackermann turn angular_velocity
    double distFromLine = distToTarget * (1-cos(angleToTarget));

    // Debug
    std::cout << err_straightLine /M_PI*180  << " Error if by straight line (deg)" << std::endl;
    std::cout << err_ackermann /M_PI*180  << " Error if by Ackermann (deg)" << std::endl;
    std::cout << distFromLine <<  " Distance from straight line" << std::endl;
    // SELECT THE MORE APPROPRIATE MOTION - WHICH ALIGNS THE ROBOT MORE WITH THE TARGET Orientation
    // TODO WEIGHTS CAN BE USED TO INCREASE THE COST OF A POINT TURN
    if( fabs(turn_radius) <= minTurnRadius            ||
        fabs(err_straightLine) < fabs(err_ackermann)  ||
        fabs(distFromLine)>= maxDisplacementAckermannTurn ) {

      targetTranslation = 0;
      tv = 0;
      targetRotation = angleToTarget;
      rv = theta > 0 ? 0.05 : -0.05;  // TODO add point turn rate to Config
      timeToGo = targetRotation/rv;
    } else {                                                 // ACKERMANN TURN CASE
      std::cout << "Ackermann turn case" << std::endl;
      targetRotation    = theta;
      targetTranslation = turn_radius*targetRotation;
      tv = 0.05 * sign;                                 // TODO add forward speed to Config
      rv = tv / turn_radius;
      timeToGo = targetTranslation/tv;
    }
  }
  std::cout << targetTranslation << " target translation (m)" << std::endl;
  std::cout << targetRotation / M_PI *180.0 << " target rotation (deg)" << std::endl;
  std::cout << tv << " translational vel. (m/s)" << std::endl;
  std::cout << rv / M_PI *180.0 << " rotational vel. (deg/s)" << std::endl;
  std::cout << timeToGo << " time to go (s)" << std::endl;
  std::cout << timeToGo*rv /M_PI *180.0 << " Final angle (deg)" << std::endl;
}

// Implementation of alignment at final destination
void WaypointNavigation::getAlignmentCommand(double &tv, double &rv){
  for (int i = 0; i<3; i++) {
    //not no sqrt, as both values are sqared, and > is vallid in this case
    if(curPose.cov_position(i,i) > targetPose.tol_position) {
      std::cout << "Variance of " << i << " is to high " << curPose.cov_position(i,i) << " should be smaller than " << targetPose.tol_position << std::endl;
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

  // Add current pose at the begining
  trajectory.insert( trajectory.begin(), new base::Waypoint(curPose.position,curPose.getYaw(),0,0));

  //
  // currentWaypoint = trajectory.begin();
  currentSegment = 0;

  if(!trajectory.empty()) {
    setTargetPose( *(trajectory.at(currentSegment))); // May become unused, lookahead point instead
    distanceToNext = new std::vector<double>(trajectory.size()-1);
    base::Vector3d wp;
    for (size_t i = 0; i < distanceToNext->size(); i++) {
      wp  = trajectory.at(i+1)->position; // w2 - w1
      wp -= trajectory.at(i)->position;
      wp(3) = 0;
      distanceToNext->at(i) = wp.dot(wp);
      distanceToNext->at(i) = sqrt(distanceToNext->at(i));
    }
    currentSegment = 0;
    setSegmentWaypoint(w1, currentSegment);
    setSegmentWaypoint(w2, currentSegment+1); // TODO can exceed
  } else {
    distanceToNext = new std::vector<double>();
  }
}

bool  WaypointNavigation::update(base::commands::Motion2D& mc){
  // 1) Update the current SEGMENT                            //TODO check whether is it the last
  // Select the segment such that robot is not withint immediate reach of the 2nd Waypoint
  while ( (w2-xr).squaredNorm() <= corridorSq && currentSegment < trajectory.size()-2 ){
      currentSegment++;
      setSegmentWaypoint(w1, currentSegment);
      setSegmentWaypoint(w2, currentSegment+1);
  }

  // 2) Get intersection point with the Path (should also return distance from the segment)
  base::Vector2d xi = getClosestPointOnPath();

  // 3) Get look ahead point
  // 4) Get motion command to the lookahead point
  return true;
}

base::Vector2d WaypointNavigation::getClosestPointOnPath(){
  return base::Vector2d::Zero();
}

bool WaypointNavigation::setSegmentWaypoint(base::Vector2d& waypoint, int indexSegment){
  if (indexSegment > ((int)trajectory.size()-1) || indexSegment<0)
    return false;
  waypoint << trajectory.at(indexSegment)->position(1),
              trajectory.at(indexSegment)->position(2);
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
