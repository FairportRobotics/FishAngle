// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwerveDrivePathCommand;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.GripperSubsystem;
import frc.robot.subsystem.LightingSubsystem;
import frc.robot.subsystem.swerve.SwerveDriveSubsystem;

public class RobotContainer {

    public static final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_XBOX_CONTROLLER_ID);
    public static final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_XBOX_CONTROLLER_ID);

    public static final ArmSubsystem armSubsystem = new ArmSubsystem(operatorController);
    public static final GripperSubsystem gripperSubsystem = new GripperSubsystem(operatorController);
    public static final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    public static final LightingSubsystem lightingSubsystem = new LightingSubsystem();

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
        autoChooser = new SendableChooser<String>();
        autoChooser.setDefaultOption("None", "");
        autoChooser.addOption("Move out zone", "Move_Out_Zone");
        autoChooser.addOption("Move out zone charge", "Move_Out_Zone_Charge");

        initCommands();
        configureBindings();
    }

    private void initCommands() {
        this.swerveDriveCommand = new SwerveDriveCommand();
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
            autoCommand = Commands.print("No auto command selected");
        } else {
            autoCommand = new SwerveDrivePathCommand(autoChooser.getSelected());
        }

        return autoCommand;
    }

    public enum GamePiece {
        CONE, CUBE
    }
}
