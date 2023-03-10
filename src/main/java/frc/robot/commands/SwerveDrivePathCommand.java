package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class SwerveDrivePathCommand extends SequentialCommandGroup {

    PathPlannerTrajectory traj = new PathPlannerTrajectory();

    /**
     * 
     * Follow path command. Use default maxVelocity of 4.0 and maxAcceleration of
     * 3.0
     * 
     * @param pathName the name of the path
     */
    public SwerveDrivePathCommand(String pathName) {
        this.traj = PathPlanner.loadPath(pathName, new PathConstraints(4.0, 3.0));

        addCommands(
                new InstantCommand(() -> {
                    RobotContainer.swerveDriveSubsystem.setOdometry(traj.getInitialHolonomicPose());
                }, RobotContainer.swerveDriveSubsystem),
                new PPSwerveControllerCommand(PathPlanner.loadPath(pathName, new PathConstraints(4.0, 3.0)),
                        RobotContainer.swerveDriveSubsystem::getPose, new PIDController(0.0, 0, 0),
                        new PIDController(0.0, 0, 0),
                        new PIDController(0.0, 0, 0), RobotContainer.swerveDriveSubsystem::drive, true,
                        RobotContainer.swerveDriveSubsystem));
    }

    /**
     * 
     * Follow path command. Use default maxVelocity of 4.0 and maxAcceleration of
     * 3.0
     * 
     * @param pathName the name of the path
     */
    public SwerveDrivePathCommand(PathPlannerTrajectory traj) {
        this.traj = traj;

        addCommands(
                new InstantCommand(() -> {
                    RobotContainer.swerveDriveSubsystem.setOdometry(traj.getInitialHolonomicPose());
                }, RobotContainer.swerveDriveSubsystem),
                new PPSwerveControllerCommand(traj,
                        RobotContainer.swerveDriveSubsystem::getPose, new PIDController(0.0, 0, 0),
                        new PIDController(0.0, 0, 0),
                        new PIDController(0.0, 0, 0), RobotContainer.swerveDriveSubsystem::drive, true,
                        RobotContainer.swerveDriveSubsystem));
    }

    /**
     * Follow path command.
     * 
     * @param pathName        the name of the path
     * @param pathConstraints maxVelocity and maxAcceleration
     */
    public SwerveDrivePathCommand(String pathName, PathConstraints pathConstraints) {
        this.traj = PathPlanner.loadPath(pathName, new PathConstraints(4.0, 3.0));

        addCommands(
                new InstantCommand(() -> {
                    RobotContainer.swerveDriveSubsystem.setOdometry(traj.getInitialHolonomicPose());
                }, RobotContainer.swerveDriveSubsystem),
                new PPSwerveControllerCommand(PathPlanner.loadPath(pathName, new PathConstraints(4.0, 3.0)),
                        RobotContainer.swerveDriveSubsystem::getPose, new PIDController(0.0, 0, 0),
                        new PIDController(0.0, 0, 0),
                        new PIDController(0.0, 0, 0), RobotContainer.swerveDriveSubsystem::drive, true,
                        RobotContainer.swerveDriveSubsystem));
    }

}
