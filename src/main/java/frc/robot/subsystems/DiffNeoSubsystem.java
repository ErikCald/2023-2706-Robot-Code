// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.DifferentialDriveAccelerationLimiter;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemChecker;
import frc.robot.SubsystemChecker.SubsystemType;
import frc.robot.config.Config;

public class DiffNeoSubsystem extends SubsystemBase {
    private static DiffNeoSubsystem instance;

    private DifferentialDrivePoseEstimator m_poseEstimator;
    private VisionRobotPose m_visionRobotPose;

    public static DiffNeoSubsystem getInstance() {
        if (instance == null) {
            if (Config.DIFF.ISNEOS == false) { 
                DriverStation.reportError(
                    String.format("DiffNeoSubsystem.getInstance() was called even though Config.DIFF_ISNEOS is false. RobotID: %d", Config.getRobotId()), 
                    true);
            }
            SubsystemChecker.subsystemConstructed(SubsystemType.DiffNeoSubsystem);
            instance = new DiffNeoSubsystem();
        }
        return instance;
    }

    /** Creates a new DifferentialNeoSubsystem. */
    private DiffNeoSubsystem() {

        m_poseEstimator = new DifferentialDrivePoseEstimator(null, 0, 0, null, null, null, null);
        m_visionRobotPose = new VisionRobotPose(null, null, null);
    }

    @Override
    public void periodic() {
        
        m_poseEstimator.update(null, null, 0, 0);
        m_visionRobotPose.update();

        
    }

    public void newVisionMeasurement(Pose2d robotPose, double timestamp) {
        m_poseEstimator.addVisionMeasurement(robotPose, timestamp);
    }

    public void arcadeDrive(double forward, double steering) {

    }

    public void setIdleMode(IdleMode mode) {

    } 
    
}
