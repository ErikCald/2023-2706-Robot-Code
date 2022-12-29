// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;

public class PoseEstimatorVisionCalculation {
    private final static double POSEESTIMATOR_VISION_ALLOWABLE_ERROR = 1.0;

    /**
     * Calculate the Pose2d of the robot using vision data and the gyro.
     * 
     * @param visionDistance Distance from the camera to the target in the camera's coordinate frame
     * @param visionYaw Yaw of the camera to the target in the camera's coordinate frame
     * @param heading Heading of the robot measured by the gyro
     * @param fieldToTarget Pose3d of the target in the field's coordinate frame. Use AprilTagFieldLayout.java for this.
     * @param robotToCamera Pose2d offset of the camera in the robot's coordinate frame
     * 
     * @return Pose2d of the robot in the field's coordinate frame, as calculated from vision.
     */
    private static Pose2d distAndYawToRobotPose(double visionDistance, double visionYaw, Rotation2d heading, Pose3d fieldToTarget, Pose2d robotToCamera) {
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

        // Add the XY from the field to the target and from the target to the robot center 
        Translation2d fieldToRobot = fieldToTarget.toPose2d().getTranslation().plus(targetToRobot);

        // Return the field to robot XY and set the gyro as the heading
        return new Pose2d(fieldToRobot, heading);
    }  

    // Store all the camera setups
    private CameraSetup m_cameraSetups[];

    // Set how all CameraSetups get the heading, the estimated pose and set a calculated pose.
    private Supplier<Rotation2d> m_getHeading;
    private Supplier<Pose2d> m_getPoseEstimate;
    private BiConsumer<Pose2d, Double> m_setPoseEstimator;

    // Store a field layout to get the fieldToTarget values for a given tag ID.
    private static AprilTagFieldLayout aprilTagFieldLayout;
    
    // CameraSetup class to handle any number of cameras on a robot
    private class CameraSetup {
        private final Pose2d m_cameraPose;

        private final DoubleSubscriber m_visionDistance;
        private final DoubleSubscriber m_visionYaw;
        private DoubleSubscriber m_imageTimestamp;
        private final IntegerSubscriber m_apriltagID;
    
        private double prevTimestamp;
    
        private CameraSetup(Pose2d robotToCamera, DoubleSubscriber visionDistance, DoubleSubscriber visionYaw, DoubleSubscriber imageTimestamp, IntegerSubscriber tagID) {
            m_cameraPose = robotToCamera;

            m_visionDistance = visionDistance;
            m_visionYaw = visionYaw;
            m_imageTimestamp = imageTimestamp;
            m_apriltagID = tagID;
        }

        private void update() {
            /**
             * Get Data from vision
             */ 
            double distance = m_visionDistance.get(-99); 
            double yaw = m_visionYaw.get(-99);
            double imageTimestamp = m_imageTimestamp.get(-99);
            int apriltagID = (int) m_apriltagID.get(-99);
        
            /**
             * Check if valid
             */
            if (distance == -99 ||
                yaw == -99 ||
                imageTimestamp == -99 ||
                apriltagID == -99) {

                return;
            }

            // Check if the data is new data
            if (imageTimestamp == prevTimestamp) {
                return;


            // TODO: Check if the timestamp is too old and reject it
            // } else if () {


            } else {
                // Update the previous timestamp to the new one
                prevTimestamp = imageTimestamp;
            }

            /**
             * Get fieldToTarget from AprilTagFieldLayout
             */ 
            Optional<Pose3d> fieldToTarget = aprilTagFieldLayout.getTagPose(apriltagID);
            if (fieldToTarget.isPresent() == false) {
                // AprilTag ID is not in AprilTagFieldLayout
                return;
            }

            Pose2d newPoseEstimate = distAndYawToRobotPose(
                        distance, 
                        yaw, 
                        m_getHeading.get(), 
                        fieldToTarget.get(),
                        m_cameraPose);

            /**
             * Reject pose if it's outside the field
             */
            // TODO: Code here...

            /**
             * Reject the new pose if it's different than PoseEstimator by too much
             */
            Translation2d estimateError = m_getPoseEstimate.get().getTranslation().minus(newPoseEstimate.getTranslation());

            if (Math.abs(estimateError.getX()) > POSEESTIMATOR_VISION_ALLOWABLE_ERROR ||
                Math.abs(estimateError.getY()) > POSEESTIMATOR_VISION_ALLOWABLE_ERROR
            ) {
                // log that vision + math gave a pose that was far away from the previous estimates
                return;
            }

            // Calculated and never rejected so give PoseEstimator a new vision measurement.
            m_setPoseEstimator.accept(newPoseEstimate, imageTimestamp);
        }
    }
    
    /**
     * PoseEstimatorVisionCalculation creates camera setups for all cameras.
     * 
     * When update is called, it checks the Network Table values for all cameras, 
     * rejects bad data, calculates the pose of the robot and then gives that to a PoseEstimator.
     */
    public PoseEstimatorVisionCalculation(Supplier<Rotation2d> getHeading, Supplier<Pose2d> getPoseEstimate, BiConsumer<Pose2d, Double> setPoseEstimator) {
        m_getHeading = getHeading;
        m_getPoseEstimate = getPoseEstimate;
        m_setPoseEstimator = setPoseEstimator;

        setupAprilTagFieldLayout();

        NetworkTable cameraATable = NetworkTableInstance.getDefault().getTable("VisionCameraA");
        NetworkTable cameraBTable = NetworkTableInstance.getDefault().getTable("VisionCameraB");
        
        m_cameraSetups = new CameraSetup[] {
            new CameraSetup( // Camera A
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                cameraATable.getDoubleTopic("distance").subscribe(-99),
                cameraATable.getDoubleTopic("yaw").subscribe(-99), 
                cameraATable.getDoubleTopic("imageTimestamp").subscribe(-99), 
                cameraATable.getIntegerTopic("apriltagID").subscribe(-99)),

            new CameraSetup( // Camera B
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                cameraBTable.getDoubleTopic("distance").subscribe(-99),
                cameraBTable.getDoubleTopic("yaw").subscribe(-99), 
                cameraBTable.getDoubleTopic("imageTimestamp").subscribe(-99), 
                cameraBTable.getIntegerTopic("apriltagID").subscribe(-99)),
        };
    }

    /**
     * Update each camera setup.
     * 
     * Checks Network tables for data from vision, rejects bad data, calculates 
     *      the robot pose and passes the calculated pose to a PoseEstimator.
     */
    public void update() {
        for (int i = 0; i < m_cameraSetups.length; i++) {
            m_cameraSetups[i].update();
        }
    }

    /**
     * Prepare AprilTags
     */
    private void setupAprilTagFieldLayout() {
        var fieldLayoutPath = Filesystem.getDeployDirectory().getAbsolutePath() + "\\apriltag\\layoutTesting2022.json";
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(fieldLayoutPath);
        }
        catch (IOException e) {
            DriverStation.reportWarning("AprilTag Layout not loaded! " + fieldLayoutPath, e.getStackTrace());
            aprilTagFieldLayout = new AprilTagFieldLayout(List.of(), 0, 0);
        }

        if (DriverStation.getAlliance() == Alliance.Red) {
            aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        } else {
            aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        }
    }




    /**
     * 
     * 
     * Testing the math function {@link distAndYawToRobotPose}
     * 
     * 
     */
    private static int testMath(Pose2d expected, Pose2d actual, String message) {
        System.out.printf("\n%s \nExpected: %s\n  Actual: %s \n", message, expected.toString(), actual.toString());

        Pose2d err = expected.relativeTo(actual);
        if (err.getTranslation().getNorm() < 0.011 && err.getRotation().getDegrees() < 1) {
            return 1;
        }
        return 0;
    }

    private static int testMath(double exX, double exY, double exDeg, 
                        double visionDistnace, double visionYaw, double headingDeg, 
                        double targetX, double targetY, double targetDeg, 
                        double cameraX, double cameraY, double cameraDeg, 
                        String message) {

        return testMath(new Pose2d(exX, exY, Rotation2d.fromDegrees(exDeg)), 
            distAndYawToRobotPose(
                visionDistnace, visionYaw, 
                Rotation2d.fromDegrees(headingDeg), 
                new Pose3d(new Pose2d(targetX, targetY, Rotation2d.fromDegrees(targetDeg))), 
                new Pose2d(cameraX, cameraY, Rotation2d.fromDegrees(cameraDeg))), 
            message);
    }

    public static void main(String[] args) {
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

        num += testMath(new Pose2d(3.2, 2.2, heading1), distAndYawToRobotPose(1, 0, heading1, targetA, cameraA), "Test1");
        num += testMath(new Pose2d(2.2, 2.2, heading1), distAndYawToRobotPose(1.41, 45, heading1, targetA, cameraA), "Test2");
        num += testMath(new Pose2d(4.283, 2, heading2), distAndYawToRobotPose(1.41, 0, heading2, targetA, cameraA), "Test3");
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

        num += testMath(new Pose2d(2.2, 4.2, heading3), distAndYawToRobotPose(2.24, 63.435, heading3, targetB, cameraB), "Test4");
        num += testMath(new Pose2d(2, 2.283, heading4), distAndYawToRobotPose(2.236, 71.565, heading4, targetB, cameraB), "Test5");
        num += testMath(new Pose2d(2.3, 1.2, heading5), distAndYawToRobotPose(2.5, 53.13, heading5, targetB, cameraB), "Test6");
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
}