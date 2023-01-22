// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    CANSparkMax m_motor0;
    CANSparkMax m_motor1;

    /** Creates a new ArmSubsystem. */
    public ArmSubsystem() {
        m_motor0 = new CANSparkMax(0, MotorType.kBrushless);
        m_motor1 = new CANSparkMax(0, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


    public void armCoordinates(double x, double z) {
        double[] joints = inverseKinametics(x, z);

        m_motor0.getPIDController().setReference(joints[0], ControlType.kSmartMotion);
        m_motor1.getPIDController().setReference(joints[1], ControlType.kSmartMotion, 0, calculateFeedforwardJointOne(), ArbFFUnits.kVoltage);
        
    }

    private double[] inverseKinametics(double x, double z) {
        return new double[] {x, z};
    }

    public double[] forwardKinematics() {
        return new double[] {0, 0};
    }

    private final double voltsAtHorizontalJoint1 = 3;

    private double calculateFeedforwardJointOne() {
        double joint0 = m_motor0.getEncoder().getPosition();
        double joint1 = m_motor1.getEncoder().getPosition();

        double angleFromHorzontial = joint1 - (180 - joint0);

        double voltage = voltsAtHorizontalJoint1 * Math.cos(angleFromHorzontial);


        return voltage;
    }


}
