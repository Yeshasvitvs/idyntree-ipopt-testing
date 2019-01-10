#include <iostream>

#include <iDynTree/InverseKinematics.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Model/JointState.h>
#include </home/yeshi/software/robotology-superbuild/robotology/iDynTree/src/model/include/iDynTree/Model/ModelTestUtils.h>
#include <iDynTree/ModelIO/ModelLoader.h>

using namespace std;

int main()
{
    cout << "iDynTree Ipopt Testing..." << endl;

    iDynTree::Model chain = iDynTree::getRandomChain(10, 0, true);

    // Create IK
    iDynTree::InverseKinematics ik;
    ik.setVerbosity(1);
    ik.setLinearSolverName("ma27");

    bool ok = ik.setModel(chain);

    //std::cout << "Random serial chain created is " << chain.toString();

    // Name of the targetFrame
    std::string targetFrame = "link9";

    ASSERT_IS_TRUE(ok);

    // Always express the target as cost
    ik.setDefaultTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintNone);

    // Use the requested parametrization
    ik.setRotationParametrization(iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);

    ik.setCostTolerance(1e-6);
    ik.setConstraintsTolerance(1e-7);

    // Create also a KinDyn object to perform forward kinematics for the desired values and the optimized ones
    iDynTree::KinDynComputations kinDynDes;
    ok = kinDynDes.loadRobotModel(ik.fullModel());
    ASSERT_IS_TRUE(ok);

    // Create a random vector of internal joint positions
    iDynTree::JointPosDoubleArray sRandom;
    getRandomJointPositions(sRandom, kinDynDes.model());
    std::cout << "Random initial joint positions : " << sRandom.toString();

    // Set it to the KinDyn object to get reasonable values for constraints and targets
    ok = kinDynDes.setJointPos(sRandom);
    ASSERT_IS_TRUE(ok);

    // Add fixed base frame as constraint
    iDynTree::Transform baseTransform = iDynTree::Transform::Identity();
    ok = ik.addFrameConstraint("baseLink", baseTransform);
    ASSERT_IS_TRUE(ok);

    // Add target
    ok = ik.addTarget(targetFrame, kinDynDes.getWorldTransform(targetFrame));
    ASSERT_IS_TRUE(ok);

    // Set zero initial guess for joints
    iDynTree::JointPosDoubleArray sInitial(ik.fullModel());
    ik.setFullJointsInitialCondition(&(baseTransform), &(sInitial));

    ok = ik.solve();

    ASSERT_IS_TRUE(ok);

    iDynTree::Transform baseOpt = iDynTree::Transform::Identity();
    iDynTree::JointPosDoubleArray sOpt(ik.fullModel());
    ik.getFullJointsSolution(baseOpt, sOpt);

    std::cout << "Optimized initial joint positions : " << sOpt.toString() << std::endl;

    // Dummy variables
    iDynTree::Twist dummyVel;
    dummyVel.zero();
    iDynTree::Vector3 dummyGrav;
    dummyGrav.zero();
    iDynTree::JointDOFsDoubleArray dummyJointVel(ik.fullModel());
    dummyJointVel.zero();

    // Create a new KinDyn object with optimization solutions
    iDynTree::KinDynComputations kinDynOpt;
    ok = kinDynOpt.loadRobotModel(ik.fullModel());
    ASSERT_IS_TRUE(ok);

    kinDynOpt.setRobotState(baseOpt, sOpt, dummyVel, dummyJointVel, dummyGrav);

    double tolConstraints = 1e-6;
    double tolTargets     = 1e-3;

    // Check if the base link constraint is still valid
    ASSERT_EQUAL_TRANSFORM_TOL(kinDynOpt.getWorldTransform("baseLink"), kinDynDes.getWorldTransform("baseLink"), tolConstraints);

    // Check if the target is realized
    ASSERT_EQUAL_VECTOR_TOL(kinDynDes.getWorldTransform(targetFrame).getPosition(), kinDynOpt.getWorldTransform(targetFrame).getPosition(), tolTargets);
    ASSERT_EQUAL_MATRIX_TOL(kinDynDes.getWorldTransform(targetFrame).getRotation(), kinDynOpt.getWorldTransform(targetFrame).getRotation(), tolTargets);

    return 0;
}
