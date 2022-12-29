// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestingNT extends CommandBase {

    DoublePublisher pub;
    Timer m_timer = new Timer();

    boolean m_shouldFlush;
    boolean m_shouldLocalFlush;
    /** Creates a new TestingNT. */
    public TestingNT(boolean shouldFlush, boolean shouldLocalFlush) {
        pub = NetworkTableInstance.getDefault().getDoubleTopic("Exp_func").publish();
        m_shouldFlush = shouldFlush;
        m_shouldLocalFlush = shouldLocalFlush;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        m_timer.reset();
        m_timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        pub.set(Math.pow(7, m_timer.get()-2));
        
        if (m_shouldFlush) {
            NetworkTableInstance.getDefault().flush();
        } else if (m_shouldLocalFlush) {
            NetworkTableInstance.getDefault().flushLocal();
        }

        

        

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        m_timer.stop();
        pub.set(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.get() > 4;
    }
}
