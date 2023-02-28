package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotContainer;

public class SwerveDrivePathCommand extends PPSwerveControllerCommand {

    public SwerveDrivePathCommand(String pathName) {
        super(PathPlanner.loadPath(pathName, new PathConstraints(5.0, 3.0)),
                RobotContainer.swerveDriveSubsystem::getPose, new PIDController(0.1, 0, 0),
                new PIDController(0.1, 0, 0),
                new PIDController(0.1, 0, 0), RobotContainer.swerveDriveSubsystem::drive, true,
                RobotContainer.swerveDriveSubsystem);
    }

}
