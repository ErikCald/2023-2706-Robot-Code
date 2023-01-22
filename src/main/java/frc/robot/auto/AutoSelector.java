package frc.robot.auto;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.config.Config;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoSelector {

    

    public static Command getAutoCommand() {
        

        Map<String, Command> eventMap = new HashMap<String, Command>();
        eventMap.put("intake", new InstantCommand(() -> System.out.println("intake")));
        eventMap.put("shoot", new InstantCommand(() -> System.out.println("shoot")));
    

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            SwerveSubsystem.getInstance()::getPose, 
            SwerveSubsystem.getInstance()::resetOdometry, 
            Config.Swerve.kSwerveDriveKinematics, 
            new PIDConstants(0, 0, 0), 
            new PIDConstants(0, 0, 0), 
            SwerveSubsystem.getInstance()::setModuleStates, 
            eventMap, 
            SwerveSubsystem.getInstance());


        PathPlannerTrajectory path = PathPlanner.loadPath("RapidReactTest2", 1, 1);
    

        
        return autoBuilder.fullAuto(path);

    }
}
