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
    ik.setVerbosity(0);
    ik.setLinearSolverName("ma27");

    bool ok = ik.setModel(chain);

    std::cout << "Random serial chain created is " << chain.toString();

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
    iDynTree::JointPosDoubleArray sInitial(ik.fullModel());
    std::cout << "sInitial " << sInitial.toString();

    // Add fixed base frame as constraint
    iDynTree::Transform baseTransform = iDynTree::Transform::Identity();
    ok = ik.addFrameConstraint("baseLink", baseTransform);
    ASSERT_IS_TRUE(ok);

    // Add target
    ok = ik.addTarget(targetFrame, iDynTree::Transform::Identity());
    ASSERT_IS_TRUE(ok);


    ik.setFullJointsInitialCondition(&(baseTransform), &(sInitial));

    ok = ik.solve();

    ASSERT_IS_TRUE(ok);

    iDynTree::Transform baseOpt = iDynTree::Transform::Identity();
    iDynTree::JointPosDoubleArray sOpt(ik.fullModel());
    ik.getFullJointsSolution(baseOpt, sOpt);

    std::cout << "Joint solution is " << sOpt.toString() << std::endl;

    return 0;
}
