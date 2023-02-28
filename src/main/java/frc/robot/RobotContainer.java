// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.GripperSubsystem;
import frc.robot.subsystem.LightingSubsystem;
import frc.robot.subsystem.swerve.SwerveDriveSubsystem;
import frc.robot.subsystem.swerve.SwerveDriveSubsystem.SwerveDriveBuilder;
import frc.robot.subsystem.swerve.SwerveModule.SwerveModuleBuilder;

public class RobotContainer {

    CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_XBOX_CONTROLLER_ID);
    CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_XBOX_CONTROLLER_ID);

    public final ArmSubsystem armSubsystem = new ArmSubsystem(operatorController);
    public final GripperSubsystem gripperSubsystem = new GripperSubsystem(operatorController);
    public final SwerveDriveSubsystem swerveDriveSubsystem;
    public final LightingSubsystem lightingSubsystem = new LightingSubsystem();

    private final SendableChooser<String> autoChooser;

    private SwerveDriveCommand swerveDriveCommand;

    Trigger toggleGripperBtn;
    Trigger highPosBtn;
    Trigger midPosBtn;
    Trigger lowPosBtn;
    Trigger homePosBtn;

    Trigger gamePieceToggleBtn;

    GamePiece requestedGamePiece = GamePiece.CONE;

    public RobotContainer() {

        this.swerveDriveSubsystem = new SwerveDriveBuilder()
                .addSwerveModule(new SwerveModuleBuilder()
                        .setName("Front Left")
                        .setLocation(Constants.WHEEL_BASE / 2, Constants.TRACK_WIDTH / 2)
                        .setMotorIds(Constants.FRONT_LEFT_DRIVE_ID, Constants.FRONT_LEFT_ENCODER_ID)
                        .setAbsoluteEncoderId(Constants.FRONT_LEFT_ENCODER_ID)
                        .setSteerEncoderOffset(Constants.FRONT_LEFT_SWERVE_OFFSET)
                        .setDriveEncoderTicksPerMeter(Constants.ENCODER_TICKS_PER_METER)
                        .setSteerPID(Constants.SWERVE_STEER_P, Constants.SWERVE_STEER_I, Constants.SWERVE_STEER_D)
                        .setSwerveProfile(Constants.MAX_ANG_ACC, Constants.MAX_ANG_VEL)
                        .build())
                .addSwerveModule(new SwerveModuleBuilder()
                        .setName("Front Right")
                        .setLocation(Constants.WHEEL_BASE / 2, -Constants.TRACK_WIDTH / 2)
                        .setMotorIds(Constants.FRONT_RIGHT_DRIVE_ID, Constants.FRONT_RIGHT_ENCODER_ID)
                        .setAbsoluteEncoderId(Constants.FRONT_RIGHT_ENCODER_ID)
                        .setSteerEncoderOffset(Constants.FRONT_RIGHT_SWERVE_OFFSET)
                        .setDriveEncoderTicksPerMeter(Constants.ENCODER_TICKS_PER_METER)
                        .setSteerPID(Constants.SWERVE_STEER_P, Constants.SWERVE_STEER_I, Constants.SWERVE_STEER_D)
                        .setSwerveProfile(Constants.MAX_ANG_ACC, Constants.MAX_ANG_VEL)
                        .build())
                .addSwerveModule(new SwerveModuleBuilder()
                        .setName("Back Left")
                        .setLocation(-Constants.WHEEL_BASE / 2, Constants.TRACK_WIDTH / 2)
                        .setMotorIds(Constants.BACK_LEFT_DRIVE_ID, Constants.BACK_LEFT_ENCODER_ID)
                        .setAbsoluteEncoderId(Constants.BACK_LEFT_ENCODER_ID)
                        .setSteerEncoderOffset(Constants.BACK_LEFT_SWERVE_OFFSET)
                        .setDriveEncoderTicksPerMeter(Constants.ENCODER_TICKS_PER_METER)
                        .setSteerPID(Constants.SWERVE_STEER_P, Constants.SWERVE_STEER_I, Constants.SWERVE_STEER_D)
                        .setSwerveProfile(Constants.MAX_ANG_ACC, Constants.MAX_ANG_VEL)
                        .build())
                .addSwerveModule(new SwerveModuleBuilder()
                        .setName("Back Right")
                        .setLocation(-Constants.WHEEL_BASE / 2, -Constants.TRACK_WIDTH / 2)
                        .setMotorIds(Constants.BACK_RIGHT_DRIVE_ID, Constants.BACK_RIGHT_ENCODER_ID)
                        .setAbsoluteEncoderId(Constants.BACK_RIGHT_ENCODER_ID)
                        .setSteerEncoderOffset(Constants.BACK_RIGHT_SWERVE_OFFSET)
                        .setDriveEncoderTicksPerMeter(Constants.ENCODER_TICKS_PER_METER)
                        .setSteerPID(Constants.SWERVE_STEER_P, Constants.SWERVE_STEER_I, Constants.SWERVE_STEER_D)
                        .setSwerveProfile(Constants.MAX_ANG_ACC, Constants.MAX_ANG_VEL)
                        .build())
                .setMaxDriveSpeed(1.0)
                .setMaxRotationSpeed(0.1)
                .build();

        autoChooser = new SendableChooser<String>();
        autoChooser.setDefaultOption("None", "");
        autoChooser.addOption("Move out zone", "Move_Out_Zone");
        autoChooser.addOption("Move out zone charge", "Move_Out_Zone_Charge");

        initCommands();
        configureBindings();
    }

    private void initCommands() {
        this.swerveDriveCommand = new SwerveDriveCommand(driverController, swerveDriveSubsystem);
    }

    private void configureBindings() {
        toggleGripperBtn = operatorController.leftBumper();
        toggleGripperBtn.onTrue(Commands.runOnce(() -> {
            gripperSubsystem.toggleGripper();
        }, gripperSubsystem));

        highPosBtn = operatorController.a();
        highPosBtn.onTrue(Commands.runOnce(() -> {
            double armPos = 0.0;
            double wristPos = 0.0;

            if (requestedGamePiece == GamePiece.CONE) {
                armPos = Constants.ARM_HIGH_POS_VAL_CONE;
                wristPos = Constants.WRIST_HIGH_POS_VAL_CONE;
            }
            armPos = Constants.ARM_HIGH_POS_VAL_CUBE;
            wristPos = Constants.WRIST_HIGH_POS_VAL_CUBE;

            armSubsystem.setArmPosition(armPos);
            armSubsystem.setWristPosition(wristPos);
        }, armSubsystem));

        midPosBtn = operatorController.b();
        midPosBtn.onTrue(Commands.runOnce(() -> {
            double armPos = 0.0;
            double wristPos = 0.0;

            if (requestedGamePiece == GamePiece.CONE) {
                armPos = Constants.ARM_MID_POS_VAL_CONE;
                wristPos = Constants.WRIST_MID_POS_VAL_CONE;
            }
            armPos = Constants.ARM_MID_POS_VAL_CUBE;
            wristPos = Constants.WRIST_MID_POS_VAL_CUBE;

            armSubsystem.setArmPosition(armPos);
            armSubsystem.setWristPosition(wristPos);
        }, armSubsystem));

        lowPosBtn = operatorController.x();
        lowPosBtn.onTrue(Commands.runOnce(() -> {
            double armPos = 0.0;
            double wristPos = 0.0;

            if (requestedGamePiece == GamePiece.CONE) {
                armPos = Constants.ARM_LOW_POS_VAL_CONE;
                wristPos = Constants.WRIST_LOW_POS_VAL_CONE;
            }
            armPos = Constants.ARM_LOW_POS_VAL_CUBE;
            wristPos = Constants.WRIST_LOW_POS_VAL_CUBE;

            armSubsystem.setArmPosition(armPos);
            armSubsystem.setWristPosition(wristPos);
        }, armSubsystem));

        homePosBtn = operatorController.y();
        homePosBtn.onTrue(Commands.runOnce(() -> {
            double armPos = Constants.ARM_HOME_POS_VAL;
            double wristPos = Constants.WRIST_HOME_POS_VAL;

            armSubsystem.setArmPosition(armPos);
            armSubsystem.setWristPosition(wristPos);
        }, armSubsystem));

        gamePieceToggleBtn = operatorController.rightBumper();
        gamePieceToggleBtn.onTrue(Commands.runOnce(() -> {
            requestedGamePiece = requestedGamePiece == GamePiece.CONE ? GamePiece.CUBE : GamePiece.CONE;
        }, lightingSubsystem));
    }

    public Command getTeleopDriveCommand() {
        return swerveDriveCommand;
    }

    public Command getAutonomousCommand() {

        Command autoCommand;

        if (autoChooser.getSelected() == "") {
            autoCommand = Commands.print("No auto command slected");
        } else {
            autoCommand = swerveDriveSubsystem.followTrajectoryCommand(
                    PathPlanner.loadPath(autoChooser.getSelected(), new PathConstraints(1, 1)), true);
        }

        return autoCommand;
    }

    public enum GamePiece {
        CONE, CUBE
    }
}
