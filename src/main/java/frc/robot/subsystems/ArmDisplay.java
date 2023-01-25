// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class ArmDisplay {
    private static ArmDisplay instance;
    public static ArmDisplay getInstance() {
        if (instance == null) {
            instance = new ArmDisplay();
        }
        return instance;
    }
    


    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 10);
    private final MechanismLigament2d m_armTower =
    m_armPivot.append(new MechanismLigament2d("ArmTower", 10, -90));
    private final MechanismLigament2d m_arm =
    m_armPivot.append(
        new MechanismLigament2d(
            "Arm",
            30,
            90,
            6,
            new Color8Bit(Color.kYellow)));

    private ArmDisplay() {
        

        // Put Mechanism 2d to SmartDashboard
        SmartDashboard.putData("Arm Sim", m_mech2d);
        m_armTower.setColor(new Color8Bit(Color.kBlue));
    }


    public void updateFromJointAngles(double angle0, double angle1) {
        angle0 = Math.toDegrees(angle0);
        angle1 = Math.toDegrees(angle1);

        // Update Joint angles
    }


}
