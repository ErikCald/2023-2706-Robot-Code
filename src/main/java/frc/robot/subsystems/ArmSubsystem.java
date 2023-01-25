// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    /*
     * Public Constants
     */
    public static final double m_arm0Mass = 5.0; // Kilograms
    public static final double m_arm0Length = Units.inchesToMeters(30);
    public static final double m_arm0Noise = 2.0 * Math.PI / 4096;
    public static final double m_arm0Reduction = 60;

    private final double voltsAtHorizontalJoint1 = 3;
    private final double encoder0PosConversion = 2 * Math.PI / m_arm0Reduction;
    private final double encoder1PosConversion = 2 * Math.PI / 24;

    CANSparkMax m_motor0;
    CANSparkMax m_motor1;

    SparkMaxPIDController m_sparkPid0;
    SparkMaxPIDController m_sparkPid1;

    RelativeEncoder m_encoder0;
    RelativeEncoder m_encoder1;

    ProfiledPIDController m_pid0;
    ProfiledPIDController m_pid1;

    private static ArmSubsystem INSTANCE;
    public static ArmSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ArmSubsystem();
        }
        return INSTANCE;
    }

    /** Creates a new ArmSubsystem. */
    public ArmSubsystem() {
        m_motor0 = new CANSparkMax(12, MotorType.kBrushless);
        m_motor1 = new CANSparkMax(13, MotorType.kBrushless);

        m_motor0.restoreFactoryDefaults();
        m_motor1.restoreFactoryDefaults();

        m_sparkPid0 = m_motor0.getPIDController();
        m_sparkPid1 = m_motor1.getPIDController();

        m_encoder0 = m_motor0.getEncoder();
        m_encoder1 = m_motor1.getEncoder();

        m_motor0.setInverted(false);
        m_motor1.setInverted(false);

        m_motor0.setSmartCurrentLimit(40);
        m_motor1.setSmartCurrentLimit(40);

        m_motor0.setIdleMode(IdleMode.kCoast);
        m_motor1.setIdleMode(IdleMode.kCoast);

        m_encoder0.setPositionConversionFactor(encoder0PosConversion);
        m_encoder1.setPositionConversionFactor(encoder1PosConversion);

        m_pid0 = new ProfiledPIDController(
            1, 0, 0, 
            new TrapezoidProfile.Constraints(0, 0));

        m_pid1 = new ProfiledPIDController(
            0, 0, 0, 
            new TrapezoidProfile.Constraints(0, 0));

    }

    @Override
    public void periodic() {
        // Update the Mechanism Arm angle based on the simulated arm angle
        ArmDisplay.getInstance().updateFromJointAngles(
            m_encoder0.getPosition(),
            m_encoder1.getPosition()
        );
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        ArmSim.getInstance().getJ0().setInput(m_motor0.getAppliedOutput() * RobotController.getBatteryVoltage()); 
        ArmSim.getInstance().getJ1().setInput(m_motor1.getAppliedOutput() * RobotController.getBatteryVoltage()); 

        // Next, we update it. The standard loop time is 20ms.
        ArmSim.getInstance().getJ0().update(0.020);
        ArmSim.getInstance().getJ1().update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        m_encoder0.setPosition(ArmSim.getInstance().getJ0().getAngleRads());
        m_encoder1.setPosition(ArmSim.getInstance().getJ1().getAngleRads());

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                ArmSim.getInstance().getJ0().getCurrentDrawAmps(),
                ArmSim.getInstance().getJ1().getCurrentDrawAmps()
        ));
    }


    public void armXZ(double x, double z) {
        x = Units.degreesToRadians(x);
        z = Units.degreesToRadians(z);

        double[] joints = inverseKinametics(x, z);

        setJointReferences(joints);
    }

    private double[] inverseKinametics(double x, double z) {
        return new double[] {x, z};
    }

    public double[] forwardKinematics() {
        return new double[] {0, 0};
    }

    private void setJointReferences(double[] joints) {
        double voltage0 = m_pid0.calculate(m_encoder0.getPosition(), joints[0]);
        double voltage1 = m_pid0.calculate(m_encoder1.getPosition(), joints[1]) +
                            calculateFeedforwardJointOne();

        m_sparkPid0.setReference(voltage0, ControlType.kVoltage);
        m_sparkPid1.setReference(voltage1, ControlType.kVoltage);
    }

    private double calculateFeedforwardJointOne() {
        double joint0 = m_motor0.getEncoder().getPosition();
        double joint1 = m_motor1.getEncoder().getPosition();

        double angleFromHorzontial = joint1 - (180 - joint0);

        double voltage = voltsAtHorizontalJoint1 * Math.cos(angleFromHorzontial);


        return voltage;
    }

    public void stopMotors() {
        m_motor0.stopMotor();
        m_motor1.stopMotor();

        // Need to set the reference to 0 for the simulation
        m_sparkPid0.setReference(0, ControlType.kVoltage);
        m_sparkPid1.setReference(0, ControlType.kVoltage);
    }
}
