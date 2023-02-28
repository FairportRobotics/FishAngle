package frc.robot.subsystem.swerve;

import java.io.File;
import java.io.FileInputStream;
import java.util.ArrayList;
import java.util.Properties;
import java.util.stream.Collectors;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class SwerveDriveSubsystem extends SubsystemBase {

    private final double MAX_SPEED_METERS_PER_SECOND;
    private final double MAX_ROTATION_SPEED_RADIANS_PER_SECOND;

    private ArrayList<SwerveModule> modules;

    private AHRS gyro;

    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;

    SendableChooser<Boolean> isDriveEnabled = new SendableChooser<Boolean>();

    /**
     * Construct a drive subsystem to drive the robot.
     * 
     * @param gyro Gyro to use to do odometry.
     */
    public SwerveDriveSubsystem(SwerveDriveBuilder builder) {

        gyro = new AHRS();

        MAX_SPEED_METERS_PER_SECOND = builder.maxSpeedMetersPerSecond;
        MAX_ROTATION_SPEED_RADIANS_PER_SECOND = builder.maxRotationSpeedRadiansPerSecond;

        this.modules = builder.modules;

        kinematics = new SwerveDriveKinematics(
                this.modules.stream().map((x) -> x.getModuleLocation()).toArray(Translation2d[]::new));

        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(-gyro.getYaw()),
                this.modules.stream().map((x) -> x.getPosition()).toArray(SwerveModulePosition[]::new));

        isDriveEnabled.setDefaultOption("Disabled", false);
        isDriveEnabled.addOption("Enabled", true);
        Robot.TESTING_TAB.add("Drive toggle", isDriveEnabled);
    }

    /**
     * Drive the robot.
     * 
     * @param forward Target forward component of the robot in m/s
     * @param strafe  Target sideways componoent of the robot in m/s
     * @param rotate  Target rotation rate of the robot in rad/s
     * @param yaw     Current gyro angle of the robot in degrees. Pass in 0 for
     *                robot oriented drive.
     */
    public void drive(double forward, double strafe, double rotate, double yaw) {
        if (DriverStation.isTest() && !isDriveEnabled.getSelected())
            return;
        ChassisSpeeds velocity = ChassisSpeeds.fromFieldRelativeSpeeds(forward * MAX_SPEED_METERS_PER_SECOND,
                strafe * MAX_SPEED_METERS_PER_SECOND, rotate * MAX_ROTATION_SPEED_RADIANS_PER_SECOND,
                Rotation2d.fromDegrees(-yaw));
        setModuleStates(velocity);
    }

    public void setModuleStates(ChassisSpeeds velocity) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(velocity);
        for (int i = 0; i < moduleStates.length; i++) {
            this.modules.get(i).fromModuleState(moduleStates[i]);
        }
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(-gyro.getYaw()),
                this.modules.stream().map((x) -> x.getPosition()).toArray(SwerveModulePosition[]::new));

        if (DriverStation.isTest()) {
            Robot.DEBUG_TAB.add("Odometry X Meters", odometry.getPoseMeters().getX());
            Robot.DEBUG_TAB.add("Odometry Y Meters", odometry.getPoseMeters().getY());
        }
    }

    /**
     * Get the current pose from the odometry.
     * 
     * @return current pose.
     */
    public Pose2d getPosition() {
        return odometry.getPoseMeters();
    }

    /**
     * Get the current yaw of the gyroscope.
     * 
     * @return current yaw in degrees.
     */
    public double getYaw() {
        return gyro.getYaw();
    }

    /**
     * Get the current pitch of the gyroscope.
     * 
     * @return current pitch in degrees.
     */
    public float getPitch() {
        return gyro.getPitch();
    }

    /**
     * Get the current roll of the gyroscope.
     * 
     * @return current roll in degrees.
     */
    public float getRoll() {
        return gyro.getRoll();
    }

    /**
     * Set the gyro's current direction to zero.
     */
    public void resetGyro() {
        gyro.reset();
    }

    /**
     * Approximation of the robot's current speed.
     * It approaches 0 when the robot's speed approaches 0
     * 
     * @return Approximation of the speed.
     */
    public double getAverageVelocity() {
        return this.modules.stream().map((x) -> Math.abs(x.getCurrentVelocity()))
                .collect(Collectors.summingDouble(Double::doubleValue)) / this.modules.size();
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        this.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        odometry::getPoseMeters,
                        new PIDController(0, 0, 0),
                        new PIDController(0, 0, 0),
                        new PIDController(0, 0, 0),
                        this::setModuleStates,
                        true,
                        this));
    }

    private void resetOdometry(Pose2d initialHolonomicPose) {
        odometry.resetPosition(new Rotation2d(),
                this.modules.stream().map((x) -> x.getPosition()).toArray(SwerveModulePosition[]::new),
                initialHolonomicPose);
    }

    public static class SwerveDriveBuilder {

        private ArrayList<SwerveModule> modules;

        private double maxSpeedMetersPerSecond = 5.0;
        private double maxRotationSpeedRadiansPerSecond = 0.1;

        public SwerveDriveBuilder() {
            modules = new ArrayList<SwerveModule>();
        }

        public SwerveDriveBuilder addSwerveModule(SwerveModule module) {
            modules.add(module);
            return this;
        }

        public SwerveDriveBuilder setMaxDriveSpeed(double maxSpeedMetersPerSecond){
            this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
            return this;
        }

        public SwerveDriveBuilder setMaxRotationSpeed(double maxRotationSpeedRadiansPerSecond){
            this.maxRotationSpeedRadiansPerSecond = maxRotationSpeedRadiansPerSecond;
            return this;
        }

        public SwerveDriveSubsystem build() {
            return new SwerveDriveSubsystem(this);
        }

    }
}