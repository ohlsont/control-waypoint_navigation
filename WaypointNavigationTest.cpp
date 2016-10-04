#include "WaypointNavigationTest.hpp"
#include "WaypointNavigation.hpp"
#include <Eigen/Geometry>
#include <iostream>

using namespace Eigen;

int main() {
    WaypointNavigation lp;


    base::samples::RigidBodyState robotPose;
    base::Waypoint targetPose;
    targetPose.tol_position = 0.1;

    robotPose.cov_position = Eigen::Matrix3d::Identity() * 0.001;

    double tv, rv;
    std::cout << "----- Orientation tests" << std::endl << std::endl;
    robotPose.orientation.setIdentity();
    targetPose.heading = 45.0/180.0*M_PI;
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(1,1,0);
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);

    std::cout << "----- Orientation test 2 " << std::endl << std::endl;
    robotPose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(0.0/180*M_PI, Eigen::Vector3d::UnitZ()));;
    targetPose.heading = 85.0/180.0*M_PI;
    robotPose.position =  Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(1,1,0);
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);

    /**/
    // NEW tests
    //test case target in front of robot
    std::cout << "----- FWD TEST" << std::endl << std::endl;
    robotPose.orientation.setIdentity();
    targetPose.heading = 0;
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(1,0,0);
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    assert(tv > 0 && fabs(rv) < 0.001);
    std::cout << "Test 1 PASSED" << std::endl << std::endl;
    // NEW tests
    //test case target in front of robot
    std::cout << "----- BWD TEST" << std::endl << std::endl;
    robotPose.orientation.setIdentity();
    targetPose.heading = 0;
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(-1,0,0);
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    assert(tv < 0 && fabs(rv) < 0.001);
    std::cout << "Test 2 PASSED" << std::endl << std::endl;


    robotPose.orientation.setIdentity();
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.heading = 0;
    // Ackermann to 1st Quadrant
    std::cout << "----- ACKERMANN 1q" << std::endl << std::endl;
    targetPose.heading = 80.0/180.0*M_PI;
    targetPose.position = Eigen::Vector3d(1,1,0);
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    assert(tv > 0 && rv > 0);
    std::cout << "Test 3.1 (1st Q Ackermann) PASSED" << std::endl << std::endl;
    // Ackermann to 2st Quadrant
    std::cout << "----- ACKERMANN 2q" << std::endl << std::endl;
    targetPose.heading = - 80/180.0*M_PI;
    targetPose.position = Eigen::Vector3d(-1,1,0);
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    assert(tv < 0 && rv < 0);
    std::cout << "Test 3.2 (2nd Q Ackermann) PASSED" << std::endl << std::endl;
    // Ackermann to 3rd Quadrant
    std::cout << "----- ACKERMANN 3q" << std::endl << std::endl;
    targetPose.heading = 80/180.0*M_PI;
    targetPose.position = Eigen::Vector3d(-1,-1,0);
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    assert(tv < 0 && rv > 0);
    std::cout << "Test 3.3 (3rd Q Ackermann) PASSED" << std::endl << std::endl;

    // Ackermann to 4th Quadrant
    std::cout << "----- ACKERMANN 4q" << std::endl << std::endl;
    targetPose.heading = - 80/180.0*M_PI;
    targetPose.position = Eigen::Vector3d(1,-1,0);
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    assert(tv > 0 && rv < 0);
    std::cout << "Test 3.4 (4th Q Ackermann) PASSED" << std::endl << std::endl;

    std::cout << "----- No operation " << std::endl << std::endl;
    robotPose.position = Eigen::Vector3d(0,0,0);
    robotPose.orientation.setIdentity();
    targetPose.position = Eigen::Vector3d(0,0,0);
    targetPose.heading = 0;
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    assert(fabs(tv) < 0.00001 && fabs(rv) < 0.00001);
    std::cout << "Test 4 PASSED" << std::endl << std::endl;

    std::cout << "----- Ackerman non diagonal" << std::endl << std::endl;
    robotPose.orientation.setIdentity();
    targetPose.heading = 0;
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(5,6,0);
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) > 0 && rv > 0);
    std::cout << "Test 5 PASSED" << std::endl << std::endl;
/*
    //test case robot needs to be alligned
    robotPose.orientation = Quaterniond(AngleAxisd(M_PI/4.0, Vector3d::UnitZ()));
    targetPose.heading = M_PI/2.0;
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(0,0,0);

    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv > 0);
    std::cout << "Test 7 PASSED" << std::endl << std::endl;
*/
/*
    //test case target in front of robot
    robotPose.orientation.setIdentity();
    targetPose.heading = 0;
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(1,0,0);
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(tv > 0 && fabs(rv) < 0.001);
    std::cout << "Test 1 PASSED" << std::endl << std::endl;

    //test case target left of robot
    robotPose.orientation.setIdentity();
    targetPose.heading = 0;
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(0,1,0);

    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv > 0);
    std::cout << "Test 2 PASSED" << std::endl << std::endl;

    //test case target right of robot
    robotPose.orientation.setIdentity();
    targetPose.heading = 0;
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(0,-1,0);

    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv < 0);
    std::cout << "Test 3 PASSED" << std::endl << std::endl;

    //test case target right of robot
    robotPose.orientation.setIdentity();
    targetPose.heading = 0;
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(.1,-1,0);

    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv < 0);
    std::cout << "Test 4 PASSED" << std::endl << std::endl;

    //test case target equals own position
    robotPose.position = Eigen::Vector3d(0,0,0);
    robotPose.orientation.setIdentity();
    targetPose.position = Eigen::Vector3d(0,0,0);
    targetPose.heading = 0;
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && fabs(rv) < 0.001);
    std::cout << "Test 5 PASSED" << std::endl << std::endl;


    //test case robot needs to be alligned
    robotPose.orientation.setIdentity();
    targetPose.heading = M_PI/2.0;
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(0,0,0);

    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv > 0);
    std::cout << "Test 6 PASSED" << std::endl << std::endl;

    //test case robot needs to be alligned
    robotPose.orientation = Quaterniond(AngleAxisd(M_PI/4.0, Vector3d::UnitZ()));
    targetPose.heading = M_PI/2.0;
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(0,0,0);

    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv > 0);
    std::cout << "Test 7 PASSED" << std::endl << std::endl;

    //test case robot is in front of target and rear points toward target
    robotPose.orientation.setIdentity();
    targetPose.heading = 0;
    robotPose.position = Eigen::Vector3d(1,0,0);
    targetPose.position = Eigen::Vector3d(0,0,0);
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert((fabs(tv) < 0.001) && ((rv > 0.3) || (rv < -0.3)));
    std::cout << "Test 8 PASSED" << std::endl << std::endl;
*/
}
