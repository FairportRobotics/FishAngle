// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static RobotContainer INSTANCE;
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final GripperSubsystem gripperSubsystem = new GripperSubsystem();
    public final SwerveDriveSubsystem swerveDriveSubsystem;
    public final LightingSubsystem lightingSubsystem = new LightingSubsystem();

    private SwerveDriveCommand swerveDriveCommand;

    CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_XBOX_CONTROLLER_ID);
    CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_XBOX_CONTROLLER_ID);

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
                        .setEncoderId(Constants.FRONT_LEFT_ENCODER_ID)
                        .setEncoderOffset(Constants.FRONT_LEFT_SWERVE_OFFSET)
                        .setEncoderTicksPerMeter(Constants.ENCODER_TICKS_PER_METER)
                        .setSwervePID(Constants.SWERVE_STEER_P, Constants.SWERVE_STEER_I, Constants.SWERVE_STEER_D)
                        .setDrivePID(Constants.SWERVE_DRIVE_P, Constants.SWERVE_DRIVE_I, Constants.SWERVE_DRIVE_D)
                        .setSwerveProfile(Constants.MAX_ANG_ACC, Constants.MAX_ANG_VEL)
                        .build())
                .addSwerveModule(new SwerveModuleBuilder()
                        .setName("Front Right")
                        .setLocation(Constants.WHEEL_BASE / 2, -Constants.TRACK_WIDTH / 2)
                        .setMotorIds(Constants.FRONT_RIGHT_DRIVE_ID, Constants.FRONT_RIGHT_ENCODER_ID)
                        .setEncoderId(Constants.FRONT_RIGHT_ENCODER_ID)
                        .setEncoderOffset(Constants.FRONT_RIGHT_SWERVE_OFFSET)
                        .setEncoderTicksPerMeter(Constants.ENCODER_TICKS_PER_METER)
                        .setSwervePID(Constants.SWERVE_STEER_P, Constants.SWERVE_STEER_I, Constants.SWERVE_STEER_D)
                        .setDrivePID(Constants.SWERVE_DRIVE_P, Constants.SWERVE_DRIVE_I, Constants.SWERVE_DRIVE_D)
                        .setSwerveProfile(Constants.MAX_ANG_ACC, Constants.MAX_ANG_VEL)
                        .build())
                .addSwerveModule(new SwerveModuleBuilder()
                        .setName("Back Left")
                        .setLocation(-Constants.WHEEL_BASE / 2, Constants.TRACK_WIDTH / 2)
                        .setMotorIds(Constants.BACK_LEFT_DRIVE_ID, Constants.BACK_LEFT_ENCODER_ID)
                        .setEncoderId(Constants.BACK_LEFT_ENCODER_ID)
                        .setEncoderOffset(Constants.BACK_LEFT_SWERVE_OFFSET)
                        .setEncoderTicksPerMeter(Constants.ENCODER_TICKS_PER_METER)
                        .setSwervePID(Constants.SWERVE_STEER_P, Constants.SWERVE_STEER_I, Constants.SWERVE_STEER_D)
                        .setDrivePID(Constants.SWERVE_DRIVE_P, Constants.SWERVE_DRIVE_I, Constants.SWERVE_DRIVE_D)
                        .setSwerveProfile(Constants.MAX_ANG_ACC, Constants.MAX_ANG_VEL)
                        .build())
                .addSwerveModule(new SwerveModuleBuilder()
                        .setName("Back Right")
                        .setLocation(-Constants.WHEEL_BASE / 2, -Constants.TRACK_WIDTH / 2)
                        .setMotorIds(Constants.BACK_RIGHT_DRIVE_ID, Constants.BACK_RIGHT_ENCODER_ID)
                        .setEncoderId(Constants.BACK_RIGHT_ENCODER_ID)
                        .setEncoderOffset(Constants.BACK_RIGHT_SWERVE_OFFSET)
                        .setEncoderTicksPerMeter(Constants.ENCODER_TICKS_PER_METER)
                        .setSwervePID(Constants.SWERVE_STEER_P, Constants.SWERVE_STEER_I, Constants.SWERVE_STEER_D)
                        .setDrivePID(Constants.SWERVE_DRIVE_P, Constants.SWERVE_DRIVE_I, Constants.SWERVE_DRIVE_D)
                        .setSwerveProfile(Constants.MAX_ANG_ACC, Constants.MAX_ANG_VEL)
                        .build())
                .build();

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
            armSubsystem.setArmPosition(Constants.ARM_HIGH_POS_VAL);
            armSubsystem.setWristPosition(Constants.WRIST_HIGH_POS_VAL);
        }, armSubsystem));

        midPosBtn = operatorController.b();
        midPosBtn.onTrue(Commands.runOnce(() -> {
            armSubsystem.setArmPosition(Constants.ARM_MID_POS_VAL);
            armSubsystem.setWristPosition(Constants.WRIST_MID_POS_VAL);
        }, armSubsystem));

        lowPosBtn = operatorController.x();
        lowPosBtn.onTrue(Commands.runOnce(() -> {
            armSubsystem.setArmPosition(Constants.ARM_LOW_POS_VAL);
            armSubsystem.setWristPosition(Constants.WRIST_LOW_POS_VAL);
        }, armSubsystem));

        homePosBtn = operatorController.y();
        homePosBtn.onTrue(Commands.runOnce(() -> {
            armSubsystem.setArmPosition(Constants.ARM_HOME_POS_VAL);
            armSubsystem.setWristPosition(Constants.WRIST_HOME_POS_VAL);
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
        return Commands.print("No autonomous command configured");
    }

    public enum GamePiece {
        CONE, CUBE
    }
}
