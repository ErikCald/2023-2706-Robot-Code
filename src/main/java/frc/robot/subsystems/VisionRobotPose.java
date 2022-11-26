// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.TempAprilTagFiles.AprilTagFieldLayout;
import frc.robot.config.Config;

// import edu.wpi.first.wpilibj.;

/** Add your docs here. */
public class VisionRobotPose {
    private final Transform3d cameraTransform3d = new Transform3d(); // Get from config based on which camera is running 


    private Supplier<Rotation2d> m_getGyroHeading;
    private Supplier<Pose2d> m_getPoseEstimate;
    private BiConsumer<Pose2d, Double> m_setPoseEstimator; 
    
    DoubleSubscriber m_targetDistance;
    DoubleSubscriber m_targetYaw;
    DoubleSubscriber m_imageTimestamp;
    IntegerSubscriber m_apriltagID;

    private double prevTimestamp;

    public VisionRobotPose(Supplier<Rotation2d> getGyroHeading, Supplier<Pose2d> getPoseEstimate, BiConsumer<Pose2d, Double> setPoseEstimator) {
        m_getGyroHeading = getGyroHeading;
        m_getPoseEstimate = getPoseEstimate;
        m_setPoseEstimator = setPoseEstimator;

        initAprilTags();

    }

    public void update() {
        
        /**
         * Get from vision
         */ 
        double distance = m_targetDistance.get(-99); 
        double yaw = m_targetYaw.get(-99);
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

        // Check if the data is new data
        } else if (imageTimestamp == prevTimestamp) {
            return;
        } else {

            // Update the previous timestamp to the new one
            prevTimestamp = imageTimestamp;
        }

        // Get fieldToTarget from AprilTagFieldLayout
        Optional<Pose3d> aprilTagPose3d  = aprilTagFieldLayout.getTagPose(apriltagID);
        if (aprilTagPose3d.isPresent() == false) {
            // AprilTag ID is not in AprilTagFieldLayout
            return;
        }
        Pose3d fieldToTarget = aprilTagPose3d.get();
        
        // Calculate cameraToTarget, relative to camera, measured by vision team
        Translation3d cameraToTarget = new Translation3d(
            distance * Math.cos(yaw),
            distance * Math.sin(yaw),
            fieldToTarget.getZ() - cameraTransform3d.getZ());

        Pose2d robotPose = ComputerVisionUtil.estimateFieldToRobot(
            ComputerVisionUtil.estimateCameraToTarget( // estimateCameraToTarget rotates cameraToTarget into the field coordinate system
                cameraToTarget, 
                fieldToTarget, 
                m_getGyroHeading.get()),
            fieldToTarget,
            cameraTransform3d).toPose2d();

        Translation2d estimatorTranslation = m_getPoseEstimate.get().getTranslation();
        
        if (Math.abs(estimatorTranslation.getX() - robotPose.getTranslation().getX()) < Config.POSEESTIMATOR_VISION_ALLOWABLE_ERROR &&
            Math.abs(estimatorTranslation.getY() - robotPose.getTranslation().getY()) < Config.POSEESTIMATOR_VISION_ALLOWABLE_ERROR
        ) {
            m_setPoseEstimator.accept(robotPose, imageTimestamp);
        } else {
            // log that vision + math gave a pose that was far away from the previous estiments
        }

        
        
    }

    private AprilTagFieldLayout aprilTagFieldLayout; 
    private void initAprilTags() {
        
        var fieldLayoutPath = Filesystem.getDeployDirectory().getAbsolutePath() + "\\apriltag\\2022-rapidreact.json";
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(fieldLayoutPath);
        }
        catch (IOException e) {
            DriverStation.reportWarning("AprilTag Layout not loaded! " + fieldLayoutPath, e.getStackTrace());
            aprilTagFieldLayout = new AprilTagFieldLayout(List.of(), 0, 0);
        }

        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        } else {
            aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        }
    }


}
