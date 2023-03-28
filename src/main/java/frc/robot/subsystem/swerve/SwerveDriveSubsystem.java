package frc.robot.subsystem.swerve;

import java.io.IOException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fairportrobotics.frc.poe.CameraTracking.AprilTags;
import com.fairportrobotics.frc.poe.CameraTracking.RobotFieldPosition;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveDriveSubsystem extends SubsystemBase {
    private static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final Transform3d CAM_TO_ROBOT = new Transform3d(new Translation3d(-0.2413, 0.3556, 0),
            new Rotation3d(0, 0, 0));

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

    private final SwerveDrivePoseEstimator poseEstimator;

    private RobotFieldPosition fieldPositionEstimator;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private SlewRateLimiter simVelocityX;
    private SlewRateLimiter simVelocityY;

    private AprilTags aprilTags;

    private CommandXboxController controller;

    private AnalogInput ultrasonicSensor;

    private AprilTagFieldLayout layout;

    private static final MechanicalConfiguration MK4I_L1 = new MechanicalConfiguration(
            0.10033,
            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
            false,
            (14.0 / 50.0) * (10.0 / 60.0),
            false);

    public SwerveDriveSubsystem(CommandXboxController controller) {
        this.controller = controller;

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        try {
            layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

            this.fieldPositionEstimator = new RobotFieldPosition("front-cam", CAM_TO_ROBOT, layout,
                    PoseStrategy.MULTI_TAG_PNP);
            this.fieldPositionEstimator.getPhotonPoseEstimator().setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            e.printStackTrace();
        }

        swerveModules[0] = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
                .withGearRatio(MK4I_L1)
                .withDriveMotor(MotorType.FALCON, Constants.SWERVE_DRIVE_IDS[0])
                .withSteerMotor(MotorType.FALCON, Constants.SWERVE_STEER_IDS[0])
                .withSteerEncoderPort(Constants.SWERVE_ENCODER_IDS[0])
                .withSteerOffset(Constants.SWERVE_OFFSETS[0])
                .build();

        swerveModules[1] = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0))
                .withGearRatio(MK4I_L1)
                .withDriveMotor(MotorType.FALCON, Constants.SWERVE_DRIVE_IDS[1])
                .withSteerMotor(MotorType.FALCON, Constants.SWERVE_STEER_IDS[1])
                .withSteerEncoderPort(Constants.SWERVE_ENCODER_IDS[1])
                .withSteerOffset(Constants.SWERVE_OFFSETS[1])
                .build();

        swerveModules[2] = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0))
                .withGearRatio(MK4I_L1)
                .withDriveMotor(MotorType.FALCON, Constants.SWERVE_DRIVE_IDS[2])
                .withSteerMotor(MotorType.FALCON, Constants.SWERVE_STEER_IDS[2])
                .withSteerEncoderPort(Constants.SWERVE_ENCODER_IDS[2])
                .withSteerOffset(Constants.SWERVE_OFFSETS[2])
                .build();

        swerveModules[3] = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0))
                .withGearRatio(MK4I_L1)
                .withDriveMotor(MotorType.FALCON, Constants.SWERVE_DRIVE_IDS[3])
                .withSteerMotor(MotorType.FALCON, Constants.SWERVE_STEER_IDS[3])
                .withSteerEncoderPort(Constants.SWERVE_ENCODER_IDS[3])
                .withSteerOffset(Constants.SWERVE_OFFSETS[3])
                .build();

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getHeading(),
                getModulePositions(),
                new Pose2d());

        // aprilTags = new AprilTags("front-facing");

        ultrasonicSensor = new AnalogInput(Constants.ULTRASONIC_SENSOR_ID);

        simVelocityX = new SlewRateLimiter(10);
        simVelocityY = new SlewRateLimiter(10);
    }

    public void zeroGyroscope() {
        poseEstimator.resetPosition(getHeading(),
                getModulePositions(),
                new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(),
                        Rotation2d.fromDegrees(0.0)));
    }

    public void resetOdometry(Pose2d newPose2d) {
        poseEstimator.resetPosition(getHeading(),
                getModulePositions(),
                newPose2d);
    }

    public Rotation2d getRotation() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;

        switch (RobotContainer.armSubsystem.getArmPosition()) {
            case kFolded:
            case kHome:
            case kLow:
            case kMid:
            case kHigh:
                break;
            case kStation:
                if (ultrasonicSensor.getValue() <= Constants.SUBSTATION_DISTANCE &&
                        chassisSpeeds.vyMetersPerSecond >= 0) {
                    chassisSpeeds.vyMetersPerSecond = 0;
                }
                break;
        }

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

    public AHRS getGyro() {
        return gyroscope;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] { swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(), swerveModules[3].getPosition() };
    }

    @Override
    public void periodic() {

        // if (DriverStation.getAlliance() == Alliance.Red) {
        // layout.setOrigin(OriginPosition.kRedAllianceWallRightSide); // Uncomment when
        // // on red alliance
        // } else {
        // layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        // }

        poseEstimator.update(getHeading(),
                getModulePositions());

        fieldPositionEstimator.getPhotonPoseEstimator().setReferencePose(poseEstimator.getEstimatedPosition());
        Optional<EstimatedRobotPose> cameraPose = fieldPositionEstimator.getEstimatedGlobalPose();

        Logger.getInstance().recordOutput("PhotonVision has target", cameraPose.isPresent());

        if (cameraPose.isPresent()) {
            // Only use position if lowest target ambiguity is < 0.2
            double lowestAmbiguity = 1;
            for (PhotonTrackedTarget target : cameraPose.get().targetsUsed) {
                if (target.getPoseAmbiguity() < lowestAmbiguity) {
                    lowestAmbiguity = target.getPoseAmbiguity();
                }
            }

            poseEstimator.addVisionMeasurement(cameraPose.get().estimatedPose.toPose2d(),
                    Timer.getFPGATimestamp());
            Logger.getInstance().recordOutput("PhotonVision Field Position",
                    cameraPose.get().estimatedPose);
            // if (lowestAmbiguity < 0.2) {
            // poseEstimator.addVisionMeasurement(cameraPose.get().estimatedPose.toPose2d(),
            // Timer.getFPGATimestamp());
            // Logger.getInstance().recordOutput("PhotonVision Field Position",
            // cameraPose.get().estimatedPose);
            // }
        }

        // Within distance to collect a cone or cube from substation
        // if (aprilTags.hasTargets() && (aprilTags.getClosestTarget().getFiducialId()
        // == 5
        // || aprilTags.getClosestTarget().getFiducialId() == 4)) {
        // Transform3d robotToCam = aprilTags.getClosestTarget().getBestCameraToTarget()
        // .plus(CAM_TO_ROBOT.inverse());
        // if (robotToCam.getX() < 1.5 && Math.abs(robotToCam.getY()) < 1.31445) {
        // Logger.getInstance().recordOutput("Robot Within Substation Dist", true);
        // controller.getHID().setRumble(RumbleType.kBothRumble, 1);
        // if (prevColor.equals("")) {
        // prevColor = RobotContainer.lightingSubsystem.getColor();
        // }
        // RobotContainer.lightingSubsystem.setColor("000255000");
        // } else {
        // Logger.getInstance().recordOutput("Robot Within Substation Dist", false);
        // controller.getHID().setRumble(RumbleType.kBothRumble, 0);
        // // if(!prevColor.equals("rainbow")) {
        // // RobotContainer.lightingSubsystem.rainbow(); }
        // // else if(!prevColor.equals("")) {
        // // RobotContainer.lightingSubsystem.setColor(prevColor); }
        // }
        // } else {
        // Pose2d aprilTagFourPos = new Pose2d(new Translation2d(16.178784, 6.749796),
        // new Rotation2d());
        // Pose2d aprilTagFivePos = new Pose2d(new Translation2d(0.36195, 6.749796),
        // new Rotation2d(Math.toRadians(180)));
        // Transform2d distToFour =
        // poseEstimator.getEstimatedPosition().minus(aprilTagFourPos);
        // Transform2d distToFive =
        // poseEstimator.getEstimatedPosition().minus(aprilTagFivePos);

        // if (Math.abs(distToFour.getX()) < 1.5 &&
        // Math.abs(distToFour.getTranslation().getY()) < 1.31445
        // && aprilTagFourPos.getRotation()
        // .getDegrees() > poseEstimator.getEstimatedPosition()
        // .getRotation().getDegrees() - 25
        // && aprilTagFourPos.getRotation().getDegrees() < poseEstimator
        // .getEstimatedPosition().getRotation().getDegrees() + 25) {
        // Logger.getInstance().recordOutput("Robot Within Substation Dist", true);
        // controller.getHID().setRumble(RumbleType.kBothRumble, 1);
        // // if(prevColor.equals("")) { prevColor =
        // // RobotContainer.lightingSubsystem.getColor(); }
        // // RobotContainer.lightingSubsystem.setColor("000255000");
        // } else if (Math.abs(distToFive.getX()) < 1.5
        // && Math.abs(distToFive.getTranslation().getY()) < 1.31445
        // && aprilTagFivePos.getRotation()
        // .getDegrees() > poseEstimator.getEstimatedPosition()
        // .getRotation().getDegrees() - 25
        // && aprilTagFivePos.getRotation().getDegrees() < poseEstimator
        // .getEstimatedPosition().getRotation().getDegrees() + 25) {
        // Logger.getInstance().recordOutput("Robot Within Substation Dist", true);
        // controller.getHID().setRumble(RumbleType.kBothRumble, 1);
        // if (prevColor.equals("")) {
        // prevColor = RobotContainer.lightingSubsystem.getColor();
        // }
        // // RobotContainer.lightingSubsystem.setColor("00025500");
        // } else {
        // Logger.getInstance().recordOutput("Robot Within Substation Dist", false);
        // controller.getHID().setRumble(RumbleType.kBothRumble, 0);
        // // if(!prevColor.equals("rainbow")) {
        // // RobotContainer.lightingSubsystem.rainbow(); }
        // // else if(!prevColor.equals("")) {
        // // RobotContainer.lightingSubsystem.setColor(prevColor); }
        // }

        // }

        // Figure out if we can drop a cone or cube into the grid (with diff levels)
        /*
         * double blueAllianceGridX = 1.396492;
         * double redAllianceGridX = 15.144496;
         * double gridY = 5.491226;
         * double mid = 7.90829;
         * if(getPose().getY() <= 5.491226 && getPose().getY() > 0) {
         * if((getPose().getX() < 4.90829 && getPose().getRotation().getDegrees() > 270
         * && getPose().getRotation().getDegrees() < 30) ||
         * (getPose().getX() > 4.90829 && getPose().getRotation().getDegrees() > 150 &&
         * getPose().getRotation().getDegrees() < 210)) {
         * if(getPose().getX() <= blueAllianceGridX + 0.215138 || getPose().getX() >=
         * redAllianceGridX - 0.215138) {
         * Logger.getInstance().recordOutput("Robot Grid Placement", "Top");
         * } else if(getPose().getX() <= blueAllianceGridX + 0.646938 ||
         * getPose().getX() >= redAllianceGridX - 0.646938) {
         * Logger.getInstance().recordOutput("Robot Grid Placement", "Middle");
         * } else if(getPose().getX() <= blueAllianceGridX + 1.016 || getPose().getX()
         * >= redAllianceGridX - 1.016) {
         * Logger.getInstance().recordOutput("Robot Grid Placement", "Bottom");
         * } else {
         * Logger.getInstance().recordOutput("Robot Grid Placement", "None");
         * }
         * } else {
         * Logger.getInstance().recordOutput("Robot Within Grid Dist", false);
         * controller.getHID().setRumble(RumbleType.kBothRumble, 0);
         * }
         * 
         * }
         * }
         */

        // Logging
        Logger.getInstance().recordOutput("Pose Estimator", poseEstimator.getEstimatedPosition());
        Logger.getInstance().recordOutput("SwerveModuleStates", moduleStates);
        Logger.getInstance().recordOutput("Ultrasonic Distance", ultrasonicSensor.getValue());

        Logger.getInstance().recordOutput("Gyro Heading",
                Math.toDegrees(gyroscope.getRotation2d().getRotations()));
        Logger.getInstance().recordOutput("Gyro Pitch", gyroscope.getRoll());
        Logger.getInstance().recordOutput("Gyro Roll", gyroscope.getPitch());
        Logger.getInstance().recordOutput("Gyro AccelX", gyroscope.getWorldLinearAccelX());
        Logger.getInstance().recordOutput("Gyro AccelY", gyroscope.getWorldLinearAccelY());
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
