// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ArmSim {
    private static ArmSim instance;

    public static ArmSim getInstance() {
        if (instance == null) {
            instance = new ArmSim();
        }
        return instance;
    }

    // Simulation classes help us simulate what's going on, including gravity.

    // This arm sim represents an arm that can travel from -75 degrees (rotated down
    // front)
    // to 255 degrees (rotated down in the back).
    private final SingleJointedArmSim m_armSim0;
    private final SingleJointedArmSim m_armSim1;

    private ArmSim() {

        m_armSim0 = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            ArmSubsystem.m_arm0Reduction,
            SingleJointedArmSim.estimateMOI(ArmSubsystem.m_arm0Length, ArmSubsystem.m_arm0Mass),
            ArmSubsystem.m_arm0Length,
            Units.degreesToRadians(-75),
            Units.degreesToRadians(255),
            ArmSubsystem.m_arm0Mass,
            true,
            VecBuilder.fill(ArmSubsystem.m_arm0Noise) // Add noise, std-dev
        );

    
        m_armSim1 = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            ArmSubsystem.m_arm0Reduction,
            SingleJointedArmSim.estimateMOI(ArmSubsystem.m_arm0Length, ArmSubsystem.m_arm0Mass),
            ArmSubsystem.m_arm0Length,
            Units.degreesToRadians(-75),
            Units.degreesToRadians(255),
            ArmSubsystem.m_arm0Mass,
            true,
            VecBuilder.fill(ArmSubsystem.m_arm0Noise) // Add noise, std-dev
        );
    }

    public SingleJointedArmSim getJ0() {
        return m_armSim0;
    }

    public SingleJointedArmSim getJ1() {
        return m_armSim1;
    }
}
