package frc.robot.subsystem.swerve;

import java.io.IOException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.fairportrobotics.frc.poe.CameraTracking.RobotFieldPosition;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDriveSubsystem extends SubsystemBase {
    private static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final AHRS gyroscope = new AHRS();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));
    private final SwerveDriveOdometry odometry;

    private RobotFieldPosition fieldPositionEstimator;
    private Pose3d lastKnownFieldPos;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public SwerveDriveSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
        lastKnownFieldPos = new Pose3d();
        try {
            this.fieldPositionEstimator = new RobotFieldPosition("cameraName", new Transform3d(),
                    AprilTagFields.k2023ChargedUp,
                    PoseStrategy.CLOSEST_TO_LAST_POSE);
        } catch (IOException e) {
            e.printStackTrace();
        }

        frontLeftModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, Constants.SWERVE_DRIVE_IDS[0])
                .withSteerMotor(MotorType.FALCON, Constants.SWERVE_STEER_IDS[0])
                .withSteerEncoderPort(Constants.SWERVE_ENCODER_IDS[0])
                .withSteerOffset(Constants.SWERVE_OFFSETS[0])
                .build();

        frontRightModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, Constants.SWERVE_DRIVE_IDS[1])
                .withSteerMotor(MotorType.FALCON, Constants.SWERVE_STEER_IDS[1])
                .withSteerEncoderPort(Constants.SWERVE_ENCODER_IDS[1])
                .withSteerOffset(Constants.SWERVE_OFFSETS[1])
                .build();

        backLeftModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, Constants.SWERVE_DRIVE_IDS[2])
                .withSteerMotor(MotorType.FALCON, Constants.SWERVE_STEER_IDS[2])
                .withSteerEncoderPort(Constants.SWERVE_ENCODER_IDS[2])
                .withSteerOffset(Constants.SWERVE_OFFSETS[2])
                .build();

        backRightModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, Constants.SWERVE_DRIVE_IDS[3])
                .withSteerMotor(MotorType.FALCON, Constants.SWERVE_STEER_IDS[3])
                .withSteerEncoderPort(Constants.SWERVE_ENCODER_IDS[3])
                .withSteerOffset(Constants.SWERVE_OFFSETS[3])
                .build();

        odometry = new SwerveDriveOdometry(
                kinematics,
                Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                new SwerveModulePosition[] { frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        backLeftModule.getPosition(), backRightModule.getPosition() });

        shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        shuffleboardTab.addNumber("Odometry Pose X", () -> odometry.getPoseMeters().getX());
        shuffleboardTab.addNumber("Odometry Pose Y", () -> odometry.getPoseMeters().getY());

        shuffleboardTab.addNumber("Camera Pose X", () -> lastKnownFieldPos.getX());
        shuffleboardTab.addNumber("Camera Pose Y", () -> lastKnownFieldPos.getY());
    }

    public void zeroGyroscope() {
        odometry.resetPosition(
                Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                new SwerveModulePosition[] { frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        backLeftModule.getPosition(), backRightModule.getPosition() },
                new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)));
    }

    public Rotation2d getRotation() {
        return odometry.getPoseMeters().getRotation();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void printOffsets() {
        System.out.println("FrontLeft: " + frontLeftModule.getSteerAngle());
        System.out.println("FrontRight: " + frontRightModule.getSteerAngle());
        System.out.println("BackLeft: " + backLeftModule.getSteerAngle());
        System.out.println("BackRight: " + backRightModule.getSteerAngle());
    }

    @Override
    public void periodic() {
        odometry.update(
                Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                new SwerveModulePosition[] { frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        backLeftModule.getPosition(), backRightModule.getPosition() });

        Optional<EstimatedRobotPose> cameraPose = fieldPositionEstimator.getEstimatedGlobalPose();

        if (cameraPose.isPresent()) {
            lastKnownFieldPos = cameraPose.get().estimatedPose;
            Logger.getInstance().recordOutput("PhotonVision Field Position", cameraPose.get().estimatedPose);
        } else {
            lastKnownFieldPos = new Pose3d(odometry.getPoseMeters());
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());

        // Logging
        Logger.getInstance().recordOutput("SwerveModuleStates", states);
        Logger.getInstance().recordOutput("Last Known Field Position", lastKnownFieldPos);
        Logger.getInstance().recordOutput("Odometry Field Position", odometry.getPoseMeters());
    }
}
