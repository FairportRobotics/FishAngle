package frc.robot.subsystem.swerve;

import java.io.IOException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.fairportrobotics.frc.poe.CameraTracking.RobotFieldPosition;
import com.fairportrobotics.frc.poe.CameraTracking.WPILIB.WPI_AprilTagPoseEstimator;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
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

        private final SwerveModule[] swerveModules = new SwerveModule[4];
        private SwerveModuleState[] moduleStates = { new SwerveModuleState(), new SwerveModuleState(),
                        new SwerveModuleState(), new SwerveModuleState() };

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
        private final SwerveDrivePoseEstimator poseEstimator;

        private RobotFieldPosition fieldPositionEstimator;
        private final WPI_AprilTagPoseEstimator fieldPoseEstimator;

        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        private SlewRateLimiter simVelocityX;
        private SlewRateLimiter simVelocityY;

        public SwerveDriveSubsystem() {

                ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
                try {
                        this.fieldPositionEstimator = new RobotFieldPosition("front-facing", new Transform3d(),
                                        AprilTagFields.k2023ChargedUp,
                                        PoseStrategy.CLOSEST_TO_LAST_POSE);
                } catch (IOException e) {
                        e.printStackTrace();
                }

                swerveModules[0] = new MkSwerveModuleBuilder()
                                .withLayout(shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                                .withDriveMotor(MotorType.FALCON, Constants.SWERVE_DRIVE_IDS[0])
                                .withSteerMotor(MotorType.FALCON, Constants.SWERVE_STEER_IDS[0])
                                .withSteerEncoderPort(Constants.SWERVE_ENCODER_IDS[0])
                                .withSteerOffset(Constants.SWERVE_OFFSETS[0])
                                .build();

                swerveModules[1] = new MkSwerveModuleBuilder()
                                .withLayout(shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                                .withDriveMotor(MotorType.FALCON, Constants.SWERVE_DRIVE_IDS[1])
                                .withSteerMotor(MotorType.FALCON, Constants.SWERVE_STEER_IDS[1])
                                .withSteerEncoderPort(Constants.SWERVE_ENCODER_IDS[1])
                                .withSteerOffset(Constants.SWERVE_OFFSETS[1])
                                .build();

                swerveModules[2] = new MkSwerveModuleBuilder()
                                .withLayout(shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                                .withDriveMotor(MotorType.FALCON, Constants.SWERVE_DRIVE_IDS[2])
                                .withSteerMotor(MotorType.FALCON, Constants.SWERVE_STEER_IDS[2])
                                .withSteerEncoderPort(Constants.SWERVE_ENCODER_IDS[2])
                                .withSteerOffset(Constants.SWERVE_OFFSETS[2])
                                .build();

                swerveModules[3] = new MkSwerveModuleBuilder()
                                .withLayout(shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                                .withDriveMotor(MotorType.FALCON, Constants.SWERVE_DRIVE_IDS[3])
                                .withSteerMotor(MotorType.FALCON, Constants.SWERVE_STEER_IDS[3])
                                .withSteerEncoderPort(Constants.SWERVE_ENCODER_IDS[3])
                                .withSteerOffset(Constants.SWERVE_OFFSETS[3])
                                .build();

                odometry = new SwerveDriveOdometry(
                                kinematics, getHeading(),
                                getModulePositions(),
                                new Pose2d(0, 0, Rotation2d.fromDegrees(180)));

                poseEstimator = new SwerveDrivePoseEstimator(kinematics, getHeading(),
                                getModulePositions(),
                                new Pose2d(0, 0, Rotation2d.fromDegrees(180)));

                UsbCamera cam = CameraServer.startAutomaticCapture();
                System.out.println(cam.getInfo().name);

                fieldPoseEstimator = new WPI_AprilTagPoseEstimator(cam);
                // fieldPoseEstimator.runDetectorPipeline();

                simVelocityX = new SlewRateLimiter(10);
                simVelocityY = new SlewRateLimiter(10);
        }

        public void zeroGyroscope() {
                odometry.resetPosition(getHeading(),
                                getModulePositions(),
                                new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)));

                poseEstimator.resetPosition(getHeading(),
                                getModulePositions(),
                                new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(),
                                                Rotation2d.fromDegrees(0.0)));
        }

        public void resetOdometry(Pose2d newPose2d) {
                odometry.resetPosition(getHeading(),
                                getModulePositions(),
                                newPose2d);
                poseEstimator.resetPosition(getHeading(),
                                getModulePositions(),
                                newPose2d);
        }

        public Rotation2d getRotation() {
                return odometry.getPoseMeters().getRotation();
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                this.chassisSpeeds = chassisSpeeds;
                moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
                for (int i = 0; i < 4; i++) {
                        SwerveModuleState.optimize(moduleStates[i],
                                        Rotation2d.fromRadians(swerveModules[i].getSteerAngle()));
                        swerveModules[i].set(
                                        moduleStates[i].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND
                                                        * MAX_VOLTAGE,
                                        moduleStates[i].angle.getRadians());
                }
        }

        public Pose2d getPose() {
                return poseEstimator.getEstimatedPosition();
        }

        public Rotation2d getHeading() {
                return gyroscope.getRotation2d();
        }

        public SwerveModulePosition[] getModulePositions() {
                return new SwerveModulePosition[] { swerveModules[0].getPosition(),
                                swerveModules[1].getPosition(),
                                swerveModules[2].getPosition(), swerveModules[3].getPosition() };
        }

        @Override
        public void periodic() {
                odometry.update(
                                getHeading(),
                                getModulePositions());

                poseEstimator.update(getHeading(),
                                getModulePositions());

                Optional<EstimatedRobotPose> cameraPose = fieldPositionEstimator.getEstimatedGlobalPose();

                if (cameraPose.isPresent()) {
                        poseEstimator.addVisionMeasurement(cameraPose.get().estimatedPose.toPose2d(),
                                        Timer.getFPGATimestamp());
                        Logger.getInstance().recordOutput("PhotonVision Field Position",
                                        cameraPose.get().estimatedPose);
                }

                Logger.getInstance().recordOutput("AprilTagPose", fieldPoseEstimator.getEstimatedGlobalPose());

                // Logging
                Logger.getInstance().recordOutput("Odometry Field Position", odometry.getPoseMeters());
                Logger.getInstance().recordOutput("Pose Estimator", poseEstimator.getEstimatedPosition());
                Logger.getInstance().recordOutput("Gyro Heading",
                                Math.toDegrees(gyroscope.getRotation2d().getRotations()));
                Logger.getInstance().recordOutput("SwerveModuleStates", moduleStates);
        }

        @Override
        public void simulationPeriodic() {
                double x = getPose().getX();
                double y = getPose().getY();
                Rotation2d rotation = getRotation();

                double targetVelX = (chassisSpeeds.vxMetersPerSecond * Math.cos(rotation.getRadians())
                                - chassisSpeeds.vyMetersPerSecond * Math.sin(rotation.getRadians()));
                double targetVelY = (chassisSpeeds.vxMetersPerSecond * Math.sin(rotation.getRadians())
                                + chassisSpeeds.vyMetersPerSecond * Math.cos(rotation.getRadians()));
                Rotation2d dRotation = Rotation2d.fromRadians(-chassisSpeeds.omegaRadiansPerSecond * 0.02);

                x += simVelocityX.calculate(targetVelX) * 0.02;
                y += simVelocityY.calculate(targetVelY) * 0.02;
                rotation = rotation.rotateBy(dRotation);

                this.resetOdometry(new Pose2d(x, y, rotation));
        }
}
