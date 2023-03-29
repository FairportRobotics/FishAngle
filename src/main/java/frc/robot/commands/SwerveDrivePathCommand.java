package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.swerve.SwerveDriveSubsystem;

public class SwerveDrivePathCommand extends PPSwerveControllerCommand {

    PathPlannerTrajectory traj = new PathPlannerTrajectory();
    SwerveDriveSubsystem swerveDriveSubsystem;
    private boolean isFirstPath = true;

    /**
     * 
     * Follow path command. Use default maxVelocity of 4.0 and maxAcceleration of
     * 3.0
     * 
     * @param pathName the name of the path
     */
    public SwerveDrivePathCommand(String pathName, boolean isFirstPath) {
        super(PathPlanner.loadPath(pathName, new PathConstraints(2.5, 1.0)),
                RobotContainer.swerveDriveSubsystem::getPose,
                new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D),
                new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D),
                new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D),
                RobotContainer.swerveDriveSubsystem::drive, true,
                RobotContainer.swerveDriveSubsystem);
        this.traj = PathPlanner.loadPath(pathName, new PathConstraints(1.0, 1.0));
        this.isFirstPath = isFirstPath;
    }

    /**
     * 
     * Follow path command. Use default maxVelocity of 4.0 and maxAcceleration of
     * 3.0
     * 
     * @param pathName the name of the path
     */
    public SwerveDrivePathCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        super(traj,
                RobotContainer.swerveDriveSubsystem::getPose,
                new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D),
                new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D),
                new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D),
                RobotContainer.swerveDriveSubsystem::drive, true,
                RobotContainer.swerveDriveSubsystem);
        this.traj = traj;
        this.isFirstPath = isFirstPath;
    }

    /**
     * Follow path command.
     * 
     * @param pathName        the name of the path
     * @param pathConstraints maxVelocity and maxAcceleration
     */
    public SwerveDrivePathCommand(String pathName, PathConstraints pathConstraints, boolean isFirstPath) {
        super(PathPlanner.loadPath(pathName, pathConstraints),
                RobotContainer.swerveDriveSubsystem::getPose,
                new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D),
                new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D),
                new PIDController(Constants.PP_PID_P, Constants.PP_PID_I, Constants.PP_PID_D),
                RobotContainer.swerveDriveSubsystem::drive, true,
                RobotContainer.swerveDriveSubsystem);
        this.traj = PathPlanner.loadPath(pathName, pathConstraints);
        this.isFirstPath = isFirstPath;
    }

    @Override
    public void initialize() {
        PathPlannerTrajectory transformedTraj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());
        if (isFirstPath) {
            Logger.getInstance().recordOutput("PP Initial Pose", transformedTraj.getInitialHolonomicPose());
            RobotContainer.swerveDriveSubsystem.resetOdometry(transformedTraj.getInitialHolonomicPose());
        }
        super.initialize();

        setLoggingCallbacks(this::logActiveTrajectory, this::logTargetPose, this::logSetpoint, this::logError);
    }

    @Override
    public void execute() {
        super.execute();
    }

    private void logActiveTrajectory(PathPlannerTrajectory traj) {
        Logger.getInstance().recordOutput("PP Trajectory", traj);
    }

    private void logTargetPose(Pose2d pose) {
        Logger.getInstance().recordOutput("PP Target Pose", pose);
    }

    private void logSetpoint(ChassisSpeeds chassisSpeeds) {
        Logger.getInstance().recordOutput("PP ChassisSpeed X(Meters)", chassisSpeeds.vxMetersPerSecond);
        Logger.getInstance().recordOutput("PP ChassisSpeed Y(Meters)", chassisSpeeds.vyMetersPerSecond);
        Logger.getInstance().recordOutput("PP ChassisSpeed Rot(Radians)", chassisSpeeds.omegaRadiansPerSecond);
    }

    private void logError(Translation2d trans, Rotation2d rot) {
        Logger.getInstance().recordOutput("PP Translation Error X(Meters)", trans.getX());
        Logger.getInstance().recordOutput("PP Translation Error Y(Meters)", trans.getY());
        Logger.getInstance().recordOutput("PP Rotation Error(Radians)", rot.getRadians());
    }

}
