// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class EstimatorMathTesting {


    public static void main(String[] args) {
        new EstimatorMathTesting();
    }
    
    public EstimatorMathTesting() {
        int num = 0;
        int numTests = 0;

        Pose3d targetA = new Pose3d(new Pose2d(new Translation2d(3, 1), Rotation2d.fromDegrees(90)));
       
        Pose2d cameraA = new Pose2d(
            new Translation2d(
                0.2,
                -0.2),
            new Rotation2d(0));

        Rotation2d heading1 = Rotation2d.fromDegrees(-90);
        Rotation2d heading2 = Rotation2d.fromDegrees(-135);

        num += testMath(new Pose2d(3.2, 2.2, heading1), EstimatorMathTesting.distAndYawToRobotPose(1, 0, heading1, targetA, cameraA), "Test1");
        num += testMath(new Pose2d(2.2, 2.2, heading1), EstimatorMathTesting.distAndYawToRobotPose(1.41, 45, heading1, targetA, cameraA), "Test2");
        num += testMath(new Pose2d(4.283, 2, heading2), EstimatorMathTesting.distAndYawToRobotPose(1.41, 0, heading2, targetA, cameraA), "Test3");
        numTests += 3;

        // New Target and Cam location
        Pose3d targetB = new Pose3d(new Pose2d(new Translation2d(4, 3), Rotation2d.fromDegrees(180)));

        Pose2d cameraB = new Pose2d(
            new Translation2d(-0.2, -0.2),
            Rotation2d.fromDegrees(-90)
        );

        Rotation2d heading3 = Rotation2d.fromDegrees(0);
        Rotation2d heading4 = Rotation2d.fromDegrees(45);
        Rotation2d heading5 = Rotation2d.fromDegrees(90);

        num += testMath(new Pose2d(2.2, 4.2, heading3), EstimatorMathTesting.distAndYawToRobotPose(2.24, 63.435, heading3, targetB, cameraB), "Test4");
        num += testMath(new Pose2d(2, 2.283, heading4), EstimatorMathTesting.distAndYawToRobotPose(2.236, 71.565, heading4, targetB, cameraB), "Test5");
        num += testMath(new Pose2d(2.3, 1.2, heading5), EstimatorMathTesting.distAndYawToRobotPose(2.5, 53.13, heading5, targetB, cameraB), "Test6");
        numTests += 3;

        // Geogebra Tests
        // The images for these numbers can be found here: https://drive.google.com/drive/folders/1pub6Z9ZbT0s7XFI987mB8Fsn-5cYFxbi?usp=share_link
        num += testMath(10, 4, 76.42, 8.8, 344.4, 76.42, 4, 12, 270, 0.81, 0.88, 66.42, "Test7");
        num += testMath(2.5, 3.733, 30, 9.099, 338.3, 30, 4, 12, 270, 0.54, -1.26, 79.15, "Test8");
        num += testMath(5.0782, 7.2447, 346.072, 12.609, 347.683, 346.072, 18, 10, 180, 0.4533, 0.4516, 37.339, "Test9");
        num += testMath(12.239, 12.786, 210.335, 7.75, 323.59, 210.335, 14.378, 4.364, 140.86, -0.26116, 1.0746, 106.27, "Test10");
        num += testMath(7.465, 10.427, 142.4, 7.717, 341.475, 142.4, 14.38, 4.364, 140.86, -1.477, 0.0748, 194.715, "Test11");
        num += testMath(8, 12, 90, 9.6225, 14.8683, 90, 11.4704, 1.977, 122, -0.7225, -1, 180, "Test12");
        numTests += 6;


        System.out.printf("\n\n Num failed: %d  \n  Num tests: %d", numTests - num, numTests);
    }

    // public Pose2d estimatorMath(double visionDistance, double visionYaw, Rotation2d heading, Pose3d fieldToTarget, Transform3d robotToCamera) {
    //     return estimatorMath(visionDistance, visionYaw, heading, fieldToTarget, 
    //         new Pose2d(
    //             new Translation2d(robotToCamera.getTranslation().getX(), robotToCamera.getTranslation().getY()),
    //             new Rotation2d(robotToCamera.getRotation().getZ())
    //         ));
    // }

    public static Pose2d distAndYawToRobotPose(double visionDistance, double visionYaw, Rotation2d heading, Pose3d fieldToTarget, Pose2d robotToCamera) {
        // Convert distance and yaw to XY
        Translation2d visionXY = new Translation2d(visionDistance, Rotation2d.fromDegrees(visionYaw));

        // Rotate XY from the camera's coordinate frame to the robot's coordinate frame
        Translation2d cameraToTargetRELATIVE = visionXY.rotateBy(robotToCamera.getRotation());

        // Add the XY from the robot center to camera and from the camera to the target
        Translation2d robotToTargetRELATIVE = robotToCamera.getTranslation().plus(cameraToTargetRELATIVE);

        // Rotate XY from the robot's coordinate frame to the field's coordinate frame and reverse the direction of the XY
        Translation2d targetToRobot = robotToTargetRELATIVE.rotateBy(
            heading
            ).unaryMinus();

        // Add the XY from the target to the robot center and from the field to the target
        Translation2d fieldToRobot = fieldToTarget.toPose2d().getTranslation().plus(targetToRobot);

        // Return the field to robot XY and set the gyro as the heading
        return new Pose2d(fieldToRobot, heading);
    }

    public int testMath(Pose2d expected, Pose2d actual, String message) {
        System.out.printf("\n%s \nExpected: %s\n  Actual: %s \n", message, expected.toString(), actual.toString());

        Pose2d err = expected.relativeTo(actual);
        if (err.getTranslation().getNorm() < 0.01 && err.getRotation().getDegrees() < 1) {
            return 1;
        }
        return 0;
    }


    public int testMath(double exX, double exY, double exDeg, 
                        double visionDistnace, double visionYaw, double headingDeg, 
                        double targetX, double targetY, double targetDeg, 
                        double cameraX, double cameraY, double cameraDeg, 
                        String message) {

        return testMath(new Pose2d(exX, exY, Rotation2d.fromDegrees(exDeg)), 
            EstimatorMathTesting.distAndYawToRobotPose(
                visionDistnace, visionYaw, 
                Rotation2d.fromDegrees(headingDeg), 
                new Pose3d(new Pose2d(targetX, targetY, Rotation2d.fromDegrees(targetDeg))), 
                new Pose2d(cameraX, cameraY, Rotation2d.fromDegrees(cameraDeg))), 
            message);
    }
}
