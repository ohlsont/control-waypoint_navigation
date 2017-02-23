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

#include "WaypointNavigation.hpp"
#include <iostream>

namespace waypoint_navigation_lib{

WaypointNavigation::WaypointNavigation()
{
    mNavigationState = TARGET_REACHED;
    // Booleans
    targetSet   = false;
    poseSet     = false;
    finalPhase  = false;
    first_waypoint_reached = false;

    // Ackermann turn parameters
    minTurnRadius    = 0.6; // (in meters)
    maxDisplacementAckermannTurn = 0.25; // (meters from straight line to the next point)
    
    // Alignment parameters
    alignment_deadband   = 5.0 / 180.0 * M_PI;
    alignment_saturation = 20.0 / 180.0 * M_PI;
    rotationalVelocity   = 10.0 / 180.0 * M_PI;    // [rad/s] ... cca 8.6 deg/s

    defaultTolHeading	= 3.0/180.0 * M_PI;
    defaultTolPos 		= 0.075;
    headingErr = 0;
    alignment_P = rotationalVelocity/alignment_saturation;
    alignment_D = 0.75*alignment_P;

    pd_initialized = false;
    
    // Velocities
    translationalVelocity = 0.05; // [m/s]
    
    // Distances
    corridor = .2;
    lookaheadDistance = .6;
    distanceToNext = new std::vector<double>();

}

NavigationState WaypointNavigation::getNavigationState() {
    return mNavigationState;
}

void  WaypointNavigation::setNavigationState(NavigationState state){
	if (mNavigationState != state)
	{
	    // TODO: terrible fix, I should not be allowed to do this
	    if(!first_waypoint_reached && state == DRIVING)
	    {
	        mNavigationState = MOVE_TO_START;
	    }
	    else
	    {
    	    mNavigationState = state;
    	}
    	pd_initialized = false;
    }
}
double WaypointNavigation::getLookaheadDistance(){
    return lookaheadDistance;
}
// setPose:
// sets the pose with a validity check (to avoid NaNs from vicon)
// If invalid pose is received, it is discarded and false is returned
bool WaypointNavigation::setPose(base::samples::RigidBodyState& pose)
{
    if( isnan(pose.position(0)) || isnan(pose.position(1)) ){
        // Position data are not valid, pose will not be set
        if (WAYPOINT_NAVIGATION_DEBUG){
        	std::cout << "Position is not valid!" << std::endl;
    	}
        return false;
    } else {
        curPose = pose;
        xr = base::Vector2d(pose.position(0), pose.position(1));
        if (!poseSet && !trajectory.empty()){
            w1 << curPose.position(0), curPose.position(1);
            setSegmentWaypoint(w2, currentSegment);
        }
        poseSet=true;
        return true;
    }
}

void WaypointNavigation::setLookaheadPoint(base::Waypoint& waypoint)
{
    targetSet = true;
    lookaheadPoint = waypoint;
}

// Get the movement command from current pose to the current lookahead point
void WaypointNavigation::getMovementCommand (base::commands::Motion2D& mc)
{
        if(!targetSet || !poseSet) {
        	if (WAYPOINT_NAVIGATION_DEBUG){
        		std::cout << "No target or pose specified" << std::endl;
        	}
        mc.translation = 0;
        mc.rotation = 0;
        return;
    }

    // Vector from current to target position
    Eigen::Vector3d driveVector = lookaheadPoint.position - curPose.position;
    driveVector.z() = 0; // Do not care about robot's Z displacement, navigation in 2D plane
    double distToTarget = driveVector.norm();

    // Transform Lookahead point to Robot Coordinate Frame
    Eigen::Vector3d lookaheadPointRCF(0,0,0);
    lookaheadPointRCF = Eigen::AngleAxisd(-curPose.getYaw(), Eigen::Vector3d::UnitZ()) * (lookaheadPoint.position - curPose.position);

    /* Debug
    std::cout << "Robot, WCF:    \t ("  << curPose.position.x()
                                << ", " << curPose.position.y() << ") " << std::endl;
    std::cout << "Lookahead, WCF:\t ("  << lookaheadPoint.position.x()
                                << ", " << lookaheadPoint.position.y() << ") " << std::endl;
    std::cout << "Lookahead, RCF:\t ("  << lookaheadPointRCF.x()
                                << ", " << lookaheadPointRCF.y() << ") " << std::endl;
	*/

    int sign = ( lookaheadPointRCF.x() < 0 ? -1 : 1);
    if( fabs(lookaheadPointRCF.y()) <=  0.001) {                // Straight line motion if dY below [1mm]
        //std::cout << "Straight line case" << std::endl;
        mc.translation = sign * translationalVelocity;
        mc.rotation = 0;
    } else {                                      // Turn Required
        double turn_radius = (lookaheadPointRCF.x()*lookaheadPointRCF.x() + lookaheadPointRCF.y()*lookaheadPointRCF.y())
        /(2*lookaheadPointRCF.y());

        // Heading of the robot at the end of the turn to the lookahead point
        double theta = atan2(lookaheadPointRCF.y(),lookaheadPointRCF.x())*2; 
        // In Robot Coordinate Frame ... -360 to +360 deg

        wrapAngle(theta); //maps -360 to +360 deg --> 0 to 180, -180 to 180, -180 to 0
        // Angle of the turn rotation,
        // heading difference from the current heading to the lookahead direction
        targetHeading = theta/2; // In range of -90 to +90

        if (!backwardPerimtted && sign<0){
            // Robot not allowed to go backwards
            targetHeading += M_PI;
            wrapAngle(targetHeading);
        }
        // Select Ackermann or Point turn + Straigt line (THIS IS CURRENTLY UNUSED)
        // Based on which combination is closer to the target orientation
        // Minimizing the misalignment at target waypoint (presumably points towards the next waypoint)
        // double err_straightLine = - (lookaheadPoint.heading - curPose.getYaw());// Target heading in Robot Coordinate frame
        // double err_ackermann    = theta +  err_straightLine;               //  By Ackermann turn - Target heading
        // err_straightLine       += targetHeading;                          //   By Straight line  - Target heading

        // Maximum distance from the straight line to the waypoint
        // lies in 1/2 the Ackermann turn angular_velocity
        double distFromLine = distToTarget * (1-cos(targetHeading));

        /* Debug
        std::cout << err_straightLine /M_PI*180  << " Error if by straight line (deg)" << std::endl;
        std::cout << err_ackermann /M_PI*180  << " Error if by Ackermann (deg)" << std::endl;
        std::cout << distFromLine <<  " Distance from straight line (m)" << std::endl;
        std::cout   << "R turn\t =\t "
                    << turn_radius << "m" << std::endl;
        std::cout   << "Targ. heading\t =\t "
                    <<  targetHeading/M_PI*180  << " deg" << std::endl;
		*/

        // SELECT THE MORE APPROPRIATE MOTION
        if( fabs(turn_radius) <= minTurnRadius            ||
            fabs(distFromLine)>= maxDisplacementAckermannTurn )
        {
            //std::cout << "PT:\t";
            mc.translation = 0;
            mc.rotation = targetHeading > 0 ? rotationalVelocity : -rotationalVelocity;
        }
        else
        {                                                 // ACKERMANN TURN CASE
            //std::cout << "ACK:\t";
            mc.translation = sign * translationalVelocity;
            mc.rotation    = mc.translation / turn_radius;
        }
        // Transform target heading to WCF
        targetHeading += curPose.getYaw();
        wrapAngle(targetHeading);
    }
}

/*
		Sets the new trajectory and calculates the distances between consecutivewaypoints
*/
void WaypointNavigation::setTrajectory(std::vector< base::Waypoint *>& t )
{
    // Delete old trajectory
    for(std::vector<base::Waypoint *>::iterator it = trajectory.begin(); it != trajectory.end(); it++)
    {
        delete *it;
    }
    trajectory.clear();
    trajectory = t;
    targetSet  = false;
    
    if(!trajectory.empty()) {
    	// Add current pose at the begining
	    if (poseSet){
	    	initilalizeCurrentSegment();
	    } else {
	    	currentSegment = 0;
	    }
	    // Precalculate distances
        distanceToNext->resize(trajectory.size()-1);
        base::Vector3d wp;
        for (size_t i = 0; i < distanceToNext->size(); i++) {
            wp  = trajectory.at(i+1)->position; // w2 - w1
            wp -= trajectory.at(i)->position;
            wp.z() = 0;
            distanceToNext->at(i) = wp.norm();
        }
        // Check goal tolerances
        if (trajectory.back()->tol_position <= 0 || trajectory.back()->tol_heading <= 0){
        	trajectory.back()->tol_position = defaultTolPos;
        	trajectory.back()->tol_heading  = defaultTolHeading;
        }

        // Case of 0deg target heading: use the last segment heading instead
        if (trajectory.back()->heading == 0){
        	if (trajectory.size() > 1){
        		trajectory.back()->heading = atan2( wp.y(), wp.x());
        	} else { // Quickfix: Final heading from single wp trajectory.
        		wp = trajectory.back()->position - curPose.position;
        		trajectory.back()->heading = atan2( wp.y(), wp.x());
        	}
        }

        //setNavigationState(DRIVING);
        setNavigationState(MOVE_TO_START);
    } else {
        distanceToNext = new std::vector<double>();
        setNavigationState(NO_TRAJECTORY);
    }
    
}

// MAIN PATH FOLLOWING UPDATE FUNCTION CALLED FROM THE COMPONENT UPDATE HOOK
bool  WaypointNavigation::update(base::commands::Motion2D& mc){
    // 1) Update the current SEGMENT
    // Select the segment such that robot is not within the immediate reach of the 2nd Waypoint
    double distToNext = (w2-xr).norm();
    while ( distToNext <= corridor ){
        if( currentSegment < trajectory.size()-1){
            setSegmentWaypoint(w1, currentSegment);
            setSegmentWaypoint(w2, currentSegment+1);
            currentSegment++;
            distToNext = (w2-xr).norm();
        } else {
            // LAST SEGMENT HANDLING, vicinity of final waypoint
            // Executing this code means the robot is within the corridor circle of final waypoint
            double headingErr, posErr;
            distToNext = (w2-xr).norm(); // Distance to Final
            //    error = Final heading - current heading; both angles in range (-pi; pi)
            headingErr  = (trajectory.back()->heading) - curPose.getYaw();
            wrapAngle(headingErr);

            // Driving/aligning based on reaching the tolerance (the final inner radius)
            if( finalPhase || distToNext <= trajectory.back()->tol_position ){
                // Position for alignment was reached
                finalPhase = true;
                if( fabs(headingErr) < trajectory.back()->tol_heading){
                    setNavigationState(TARGET_REACHED);
                } else {
                    setNavigationState(ALIGNING); // Align to target heading
                    targetHeading = trajectory.back()->heading;
                    wrapAngle(targetHeading);
                }
            } else {
                //finalPhase = false;
                Eigen::Vector3d xf;
                xf = trajectory.back()->position;
                targetHeading = atan2( xf.y()-xr.y(), xf.x()-xr.x());       // Heading to target
                posErr = distToNext * sin(targetHeading-curPose.getYaw());
                if ( fabs(posErr) > trajectory.back()->tol_position ){
                    // This assumes straight line motion, not Ackermann, but approx valid
                    setNavigationState(ALIGNING); // with targetHeading = heading to target
                } else {
                    setNavigationState(DRIVING);
                }
            }
            if (WAYPOINT_NAVIGATION_DEBUG){
	            std::cout << "Final phase:" << finalPhase << std::endl <<
	            "\t Dist remng.   \t" << distToNext << " m" << std::endl <<
	            "\t Heading error \t" << headingErr*180/M_PI<<
	            "/" << trajectory.back()->tol_heading*180/M_PI << " deg" << std::endl <<
	            "\t Pos. err. est.\t" << posErr << "/" << trajectory.back()->tol_position <<"m " <<  std::endl;
	        }
            break;
        }
    }
    finalPhase &= (currentSegment == trajectory.size()-1);


    // 2) Get intersection point with the Path (should also return distance from the segment)
    base::Vector2d xi = getClosestPointOnPath();
    /*
    std::cout << "Closest point: (" 
    	<< xi.x() << ", " << xi.y() << ") "		<< std::endl;
    std::cout << "Dist to next: " << distToNext << std::endl;
    */

    // 3) Calculate the distance from the nominal trajectory
    distanceToPath = (xr-xi).norm();

    /*
    std::cout << "Current segment:\t"   << currentSegment   << std::endl;
    std::cout << "Dist. from nominal:\t" << distanceToPath  << std::endl;
	*/

	NavigationState currentState = getNavigationState();
    /* -------------------------------------------
    * STATEMACHINE FOR EXECUTION OF DIFFERENT PATH FOLLOWING MODES
    ------------------------------------------- */
    switch (currentState) {
        case MOVE_TO_START:
        case DRIVING:
        {
            // 0) OUT OF BOUNDARIES CHECK
            // TODO: Add an exception for the first waypoint
            if(distanceToPath >= corridor && currentState == DRIVING)
            {
                setNavigationState(OUT_OF_BOUNDARIES);
                mc.translation = 0;
                mc.rotation =0;
                return false;
            }
            else if(distanceToPath < corridor && currentState != DRIVING)
            {
                // The rover reached the first waypoint, switch to driving mode
                first_waypoint_reached = true;
                setNavigationState(DRIVING);
            }
            double distance;
            distance = distanceToPath + distToNext;

            // i) Get the look ahead point segment
            base::Vector2d lineVector, lookaheadPoint2D;
            if (WAYPOINT_NAVIGATION_DEBUG){
            	std::cout << "Distance: "<< distance << "/" << lookaheadDistance << std::endl;
            }
            if (distance > lookaheadDistance) // Lookahead within same seg.
            {
                // ii) Get the look ahead point in the current segment
                lineVector = w2-w1;
                lineVector.normalize();
                lookaheadPoint2D = xi + lineVector*(lookaheadDistance-distanceToPath);
            }
            else
            { // Find the right segment for the lookahead point
                size_t lookaheadSegment;
                lookaheadSegment = currentSegment;
                for ( ;	lookaheadSegment < distanceToNext->size() &&
                		distance <= lookaheadDistance;
                		lookaheadSegment++)
                {
                    distance +=  distanceToNext->at(lookaheadSegment);
                }

                // ii) Get the look ahead point in the lookahead segment
                setSegmentWaypoint(l1, lookaheadSegment-1);
                setSegmentWaypoint(l2, lookaheadSegment);
                lineVector = l2-l1;
                lineVector.normalize();
                lookaheadPoint2D = l2 - lineVector*(distance-lookaheadDistance);
            }
            // Set lookahead point: Vector2d -> Waypoint
            lookaheadPoint.position << lookaheadPoint2D(0),lookaheadPoint2D(1),0;
            lookaheadPoint.heading  = atan2(lineVector(1),lineVector(0));
            lookaheadPoint.tol_position = 0.1;
            targetSet = true;
            
            // iii) Get motion command to the lookahead point
            getMovementCommand(mc);
            if ( fabs(mc.translation) < 1e-6){
                setNavigationState(ALIGNING);
                // Use the Angle To Target as Target heading
            }   // Else drive using the ackermann command values
            break;
        } // --- end of DRIVING ---
        case ALIGNING:
        {
            mc.translation = 0; // Ensure
            base::Time t1 = base::Time::now();
        
            double disalignmentTolerance, headingErrPrev, headingErrDiff, alignment_dt;
            disalignmentTolerance = finalPhase ? trajectory.back()->tol_heading : alignment_deadband;
            
            headingErrPrev = headingErr;
            headingErr     = targetHeading - curPose.getYaw();
            wrapAngle(headingErr);
            saturation(headingErr,alignment_saturation);
            headingErrDiff = headingErr-headingErrPrev;

            if(pd_initialized){
            	alignment_dt = (t1-tprev).toMilliseconds()/1000.0;
            	headingErrDiff /= alignment_dt;
            	saturation(headingErrDiff,10.0/180.0*M_PI);
            	if (WAYPOINT_NAVIGATION_DEBUG){
            		std::cout << "d/dt = " << headingErrDiff*180/M_PI << "/" << alignment_dt << " = ";
            		std::cout << headingErrDiff << std::endl;
            	}
            } else {
            	pd_initialized = true;
            	headingErrDiff = 0;
            	if (WAYPOINT_NAVIGATION_DEBUG){
            		std::cout << "PD initialized" << std::endl;
            	}
            }
          
            if ( fabs(headingErr) < disalignmentTolerance){
                mc.rotation = 0;
                setNavigationState(DRIVING);
            } else {
			    mc.rotation =  alignment_P * headingErr + alignment_D * headingErrDiff;
            }

            saturation(mc.rotation,rotationalVelocity);
            tprev = t1;
            
            if (WAYPOINT_NAVIGATION_DEBUG){
	            std::cout << "Aligning:\t " << 180.0/M_PI*curPose.getYaw() << " to " 
	                << 180.0/M_PI*targetHeading  		<< "+-"   
	                << 180.0/M_PI*disalignmentTolerance << " deg,\nrv = "
	                << 180.0/M_PI*alignment_P * headingErr 		<< " + "
	                << 180.0/M_PI*alignment_D * headingErrDiff  << " ~ "
	                << 180.0/M_PI*mc.rotation    		<< "deg/s "      << std::endl;

	            std::cout   << "e-: " << 180.0/M_PI*headingErrPrev 	<< "deg." << std::endl;
	            std::cout   << "e:  " << 180.0/M_PI*headingErr 	<< "deg." << std::endl;
	            std::cout   << "de: " << 180.0/M_PI*headingErrDiff << "deg." << std::endl;
           	}

            break;
        } // --- end of ALIGNING ---
        case OUT_OF_BOUNDARIES:
        {
            mc.translation = 0;
            mc.rotation    = 0;
            double progress, distAlong, distPerpendicular;
            // Try the current segment first
            if(currentSegment > 0){
                getProgressOnSegment(currentSegment, progress, distAlong, distPerpendicular);
                if( progress > 0  && isInsideBoundaries(distAlong, distPerpendicular) )
                {
                    /*
                    std::cout << "Resolved, robot was in the current segment with progres of: "
                    << progress*100 << "\%" << std::endl;
                    */
                   setNavigationState(DRIVING);
                   break;
               	} else {
               		/*
                   std::cout << "Robot not found in the current segment corridor" << std::endl;
                   std::cout << "\t Progress (%): \t"<< progress*100 << std::endl;
                   std::cout << "\t Longitudal err (m): \t"<< distAlong << std::endl;
                   std::cout << "\t Lateral error (m): \t" << distPerpendicular << std::endl;
                   */
               	}
            }
            // Try the previous or next segment
            if(currentSegment > 1){
               getProgressOnSegment(currentSegment-1, progress, distAlong, distPerpendicular);
               	if ( isInsideBoundaries(distAlong, distPerpendicular) )
                {
                    currentSegment--;
                    setSegmentWaypoint(w1, currentSegment-1);
                    setSegmentWaypoint(w2, currentSegment);
                    setNavigationState(DRIVING);
                    // Robot is in the "Bounding box: of the previous segment
                    /*std::cout << "Resolved, robot was in previous segment with progres of: "
                    << progress*100 << "\%, segment decremented" << std::endl;
                    std:: cout << w1 << std::endl << w2 << std::endl;
                    */
                    break;
                } else {
                	/*
                    std::cout << "Robot not found in the previous segments corridor" << std::endl;
                    std::cout << "\t Progress (%): \t"<< progress*100 << std::endl;
                    std::cout << "\t Longitudal err (m): \t"<< distAlong << std::endl;
                    std::cout << "\t Lateral error (m): \t" << distPerpendicular << std::endl;
                	*/
                }
            }
            // try matching the robot position with the NEXT segment
            if(currentSegment < trajectory.size()-1)
            {
                getProgressOnSegment(currentSegment+1, progress, distAlong, distPerpendicular);
                if ( isInsideBoundaries(distAlong, distPerpendicular) )
                {
                    currentSegment++;
                    setSegmentWaypoint(w1, currentSegment-1);
                    setSegmentWaypoint(w2, currentSegment);
                    std:: cout << w1 << std::endl << w2 << std::endl;
                    setNavigationState(DRIVING);
                    /* Robot is in the "Bounding box: of the next segment /
                    std::cout << "Resolved, robot was in next segment with progres of: "
                    << progress*100 << "\%, segment incremented" << std::endl;
                    */
                    break;
                } else {
                    /*
                    std::cout << "Robot not found in the next segments corridor" << std::endl;
                    std::cout << "\t Progress (%): \t"<< progress*100 << std::endl;
                    std::cout << "\t Longitudal err (m): \t"<< distAlong << std::endl;
                    std::cout << "\t Lateral error (m): \t" << distPerpendicular << std::endl;
	  				*/                  
                }
            }
            break;
        }
        case TARGET_REACHED:
        	if (WAYPOINT_NAVIGATION_DEBUG)
            	std::cout << "Target Reached." 		<< std::endl;
            //finalPhase = false;
            break;
        case NO_TRAJECTORY:
        	if (WAYPOINT_NAVIGATION_DEBUG)
            	std::cout << "Invalid trajectory." 	<< std::endl;
            break;
        default:
        	if (WAYPOINT_NAVIGATION_DEBUG)
            	std::cout<<"Default case."<<std::endl;
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
}

bool WaypointNavigation::configure(double minR,	double tv, double rv,
    double cr, double lad, bool backwards)
    {
        std::cout << 
        "------------------------------------" << std::endl <<
        "Received Path Tracker config values:" << std::endl <<
        "Min. turn radius:\t"   << minR << " m."  << std::endl <<
        "Translat. vel.:\t\t"       << tv  << " m/s." << std::endl <<
        "Rotation. vel:\t\t"      << rv <<  " rad/s."<< std::endl <<
        "Clearance:\t\t"          << cr <<  " m."  << std::endl   <<
        "Lookahead dist.\t\t"     << lad << " m."  << std::endl   <<
        "Reverse:\t\t";
        if(backwards){
            std::cout<< "Permitted.\n";
        } else {
            std::cout<< "Forbidden.\n";
        }
        std::cout<< "------------------------------------" <<std::endl;

        // All config. parameters must be possitive
        if( minR>0 && tv>0 && rv>0 && cr > 0 && lad > 0){
            minTurnRadius           = minR;
            translationalVelocity   = tv;
            rotationalVelocity      = rv;
            corridor                = cr;
            lookaheadDistance       = lad;
            backwardPerimtted       = backwards;
            // std::cout << "Config successful, " << getLookaheadDistance() << std::endl;
            return true;
        } else {
            return false;
        }
    }

    bool WaypointNavigation::getProgressOnSegment(int segmentNumber,
        double& progress, double& distAlong, double& distPerpend){
            // Solving for parameter k such that closestPoint = w1 + k*segVector;
            // 1) segVector = (w2-w1) Vector of the segment line
            // Using the equations
            //    a) w1 + segVector*k = xi
            //    b) xr + perpendicular(segVector)*j = xi
            // This gives [xr - w1] = [perpendicular(segVector), segVector][j; k]
            // 2) Calculate k using the inverse matrix
            // 3) Calculate the point of intersection using a)
            // k is also the progress along that segment

            // Using wStart and wEnd instead of w1, w2
            // 1)
            base::Vector2d segVector, wStart, wEnd;
            setSegmentWaypoint(wStart, segmentNumber-1);
            setSegmentWaypoint(wEnd  , segmentNumber);
            segVector = wEnd-wStart;

            // 2)
            double determinant = segVector.dot(segVector);
            base::Matrix2d inverseL;
            inverseL  << segVector(1), -segVector(0),
                         segVector(0),  segVector(1);
            inverseL /= determinant;

            base::Vector2d xi = inverseL*(xr-wStart);

            // 3)
            progress =  xi(1);
            xi = wStart + progress*segVector;
            distPerpend = (xr-xi).norm();

            if ( progress >= 1 ) {
                distAlong = (progress-1)*segVector.norm();
            } else if (progress <= 0 ) {
                distAlong = progress*segVector.norm();
            } else{
                distAlong = 0;
            }
            /* // DEBUG OUTPUTS
            std::cout << "seg vector = (" << segVector(0) <<
            ", " << segVector(1) << ")" << std::endl;
            std::cout << "xr = (" << xr(0) <<
            ", " << xr(1) << ")" << std::endl;
             std::cout << "wStart = (" << wStart(0) <<
            ", " << wStart(1) << ")" << std::endl;
            std::cout << "Determinant: " << determinant << std::endl;
            std::cout << "solution = (" << xi(0) <<
            ", " << xi(1)? << ")" << std::endl;
            std::cout << inverseL << std::endl;
            */
            return true;
        }

        void WaypointNavigation::setCurrentSegment(int segmentNumber){
        	if (segmentNumber < 0){
        		if (WAYPOINT_NAVIGATION_DEBUG)
        			std::cerr << "Attemp to set invalid segment number" << std::endl;
        		return;
        	}
        	if ( segmentNumber < 1){
        		w1 << curPose.position(0), curPose.position(1);
	        	setSegmentWaypoint(w2, segmentNumber);
        	} else {
        		setSegmentWaypoint(w1, segmentNumber-1);
        		setSegmentWaypoint(w2, segmentNumber);
        	}
        	currentSegment = segmentNumber;
        }
    
        void WaypointNavigation::initilalizeCurrentSegment(){
		/* For each segment (except the zeroth (trivial) one): 
		*	a) calculate the hypothetical progress on the segment
		*  	b) store the maximum index where the robot was in corridor bounds 
		*  Set the segment to the max index found.	
		*/
        	size_t maxIndex = 0;
        	double progress, distAlong, distPerpend;
        	for (size_t i = 1; i < trajectory.size(); i++) {
        		getProgressOnSegment(i, progress, distAlong, distPerpend);
        		if ( progress >= 0 && progress <= 1
            		 && distPerpend < corridor){
            		maxIndex = i;
            	}
            	if (WAYPOINT_NAVIGATION_DEBUG){
            	std::cout << "Segment: " << i << ", progress: " << progress << 
        			", dist from nominal " << distPerpend << std::endl;
        		}
        	}
        	if (WAYPOINT_NAVIGATION_DEBUG){
        		std::cout << "Segment set to: " << maxIndex << std::endl;
        	}
        	setCurrentSegment(maxIndex);
        }

        bool WaypointNavigation::isInsideBoundaries(double& distAlong, double& distPerpend){
        	return  fabs(distAlong)		< corridor &&
                    fabs(distPerpend)	< corridor;
        }
        
        inline void WaypointNavigation::wrapAngle(double& angle){
            if ( angle > M_PI){
                angle -= 2*M_PI;
            } else if (angle < -M_PI){
                angle += 2*M_PI; 
            }
        }

        inline void WaypointNavigation::saturation(double& value, double limit){
        	if (value > limit){
        		value = limit;
        	} else if(value < -limit){
        		value = -limit;
        	}
        }

        bool WaypointNavigation::configurePD(double P, double D, double saturation){
			alignment_P = P>0 ? P : 0;
        	alignment_D = D>0 ? D : 0;
        	alignment_saturation = saturation;
        	return true;
        }

        bool WaypointNavigation::configureTol(double tolPos, double tolHeading){
        	if (tolPos>0 && tolHeading>0){
        		defaultTolPos 		= tolPos;
        		defaultTolHeading 	= tolHeading;
        		return true;
        	} else {
        		return false;
        	}
        	
        }
    


}
