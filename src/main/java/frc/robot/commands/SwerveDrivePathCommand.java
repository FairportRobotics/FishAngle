package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.swerve.SwerveDriveSubsystem;

public class SwerveDrivePathCommand extends PPSwerveControllerCommand {

    PathPlannerTrajectory traj = new PathPlannerTrajectory();
    SwerveDriveSubsystem swerveDriveSubsystem;

    /**
     * 
     * Follow path command. Use default maxVelocity of 4.0 and maxAcceleration of
     * 3.0
     * 
     * @param pathName the name of the path
     */
    public SwerveDrivePathCommand(String pathName) {
        super(PathPlanner.loadPath(pathName, new PathConstraints(1.0, 1.0)),
                RobotContainer.swerveDriveSubsystem::getPose, new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D),
                new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D),
                new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D), RobotContainer.swerveDriveSubsystem::drive, true,
                RobotContainer.swerveDriveSubsystem);
        this.traj = PathPlanner.loadPath(pathName, new PathConstraints(1.0, 1.0));
    }

    /**
     * 
     * Follow path command. Use default maxVelocity of 4.0 and maxAcceleration of
     * 3.0
     * 
     * @param pathName the name of the path
     */
    public SwerveDrivePathCommand(PathPlannerTrajectory traj) {
        super(traj,
                RobotContainer.swerveDriveSubsystem::getPose, new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D),
                new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D),
                new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D), RobotContainer.swerveDriveSubsystem::drive, true,
                RobotContainer.swerveDriveSubsystem);
        this.traj = traj;
    }

    /**
     * Follow path command.
     * 
     * @param pathName        the name of the path
     * @param pathConstraints maxVelocity and maxAcceleration
     */
    public SwerveDrivePathCommand(String pathName, PathConstraints pathConstraints) {
        super(PathPlanner.loadPath(pathName, pathConstraints),
                RobotContainer.swerveDriveSubsystem::getPose, new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D),
                new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D),
                new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D), RobotContainer.swerveDriveSubsystem::drive, true,
                RobotContainer.swerveDriveSubsystem);
        this.traj = PathPlanner.loadPath(pathName, pathConstraints);
    }

    @Override
    public void initialize() {
        super.initialize();
        RobotContainer.swerveDriveSubsystem.resetOdometry(traj.getInitialHolonomicPose());
    }

}
