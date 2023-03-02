package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotContainer;

public class SwerveDrivePathCommand extends PPSwerveControllerCommand {

    /**
     * 
     * Follow path command. Use default maxVelocity of 4.0 and maxAcceleration of 3.0
     * 
     * @param pathName the name of the path
     */
    public SwerveDrivePathCommand(String pathName) {
        super(PathPlanner.loadPath(pathName, new PathConstraints(4.0, 3.0)),
                RobotContainer.swerveDriveSubsystem::getPose, new PIDController(0.1, 0, 0),
                new PIDController(0.1, 0, 0),
                new PIDController(0.1, 0, 0), RobotContainer.swerveDriveSubsystem::drive, true,
                RobotContainer.swerveDriveSubsystem);
    }

    /**
     * Follow path command. 
     * 
     * @param pathName the name of the path
     * @param pathConstraints maxVelocity and maxAcceleration
     */
    public SwerveDrivePathCommand(String pathName, PathConstraints pathConstraints) {
        super(PathPlanner.loadPath(pathName, pathConstraints),
                RobotContainer.swerveDriveSubsystem::getPose, new PIDController(0.1, 0, 0),
                new PIDController(0.1, 0, 0),
                new PIDController(0.1, 0, 0), RobotContainer.swerveDriveSubsystem::drive, true,
                RobotContainer.swerveDriveSubsystem);
    }

}
