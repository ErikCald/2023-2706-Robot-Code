// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToVisionTarget extends CommandBase {
    private final double distanceTolarance = 0.1;
    private final double angleTolarance = 0.3;

    private final double targetHeading;

    private final ProfiledPIDController xPID;
    private final PIDController yPID;
    private final PIDController headingPID;

    private final DoubleSubscriber visionYaw;
    private final DoubleSubscriber visionDistance;

    private double targetX;
    private double targetY;

    

    /** Creates a new VisionTarget. */
    public DriveToVisionTarget(String visionYawTopicName, String visionDistanceTopicName, double desiredHeading) {
        targetHeading = desiredHeading;

        xPID = new ProfiledPIDController(
            0, 0, 0, 
            new TrapezoidProfile.Constraints(0, 0));

        yPID = new PIDController(0, 0, 0);
        headingPID = new PIDController(0, 0, 0);

        visionYaw = NetworkTableInstance.getDefault().getDoubleTopic(visionYawTopicName).subscribe(-99);
        visionDistance = NetworkTableInstance.getDefault().getDoubleTopic(visionDistanceTopicName).subscribe(-99);

        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d curPose = SwerveSubsystem.getInstance().getPose();
        double distance = visionDistance.get();
        double yaw = visionYaw.get();

        if (distance != -99) {
            targetX = curPose.getX() + distance;
        }

        if (yaw != -99) {
            targetY = curPose.getY() + distance;
        }

        double xResult = xPID.calculate(curPose.getX(), targetX);
        double yResult = yPID.calculate(curPose.getY(), targetY);
        double rotResult = headingPID.calculate(curPose.getRotation().getDegrees(), targetHeading);

        SwerveSubsystem.getInstance().drive(xResult, yResult, rotResult, true, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Pose2d currPose = SwerveSubsystem.getInstance().getPose();
        
        return 
            Math.abs(currPose.getX() - targetX) < distanceTolarance &&
            Math.abs(currPose.getY() - targetY) < distanceTolarance &&
            Math.abs(currPose.getRotation().getDegrees() - targetHeading) < angleTolarance;
    }
}
