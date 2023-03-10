// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmMoveToPositionCommand;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.ArmMoveToPositionCommand.ArmPosition;
import frc.robot.commands.GripperCommand.GripperAction;
import frc.robot.commands.autonomus.LeaveZoneAndChargeAutoCommand;
import frc.robot.commands.autonomus.LeaveZoneAutoCommand;
import frc.robot.commands.autonomus.OneConeAutoCommand;
import frc.robot.commands.autonomus.OneCubeAutoCommand;
import frc.robot.commands.autonomus.TwoCubeAutoCommand;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.GripperSubsystem;
import frc.robot.subsystem.LightingSubsystem;
import frc.robot.subsystem.swerve.SwerveDriveSubsystem;

public class RobotContainer {

    public static final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_XBOX_CONTROLLER_ID);
    public static final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_XBOX_CONTROLLER_ID);

    public static final PneumaticHub pneumaticHub = new PneumaticHub();

    public static final ArmSubsystem armSubsystem = new ArmSubsystem(operatorController);
    public static final GripperSubsystem gripperSubsystem = new GripperSubsystem(operatorController);
    public static final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    public static final LightingSubsystem lightingSubsystem = new LightingSubsystem();

    private final SendableChooser<Command> autoChooser;

    private SwerveDriveCommand swerveDriveCommand;

    Trigger toggleGripperBtn;
    Trigger highPosBtn;
    Trigger midPosBtn;
    Trigger lowPosBtn;
    Trigger homePosBtn;

    Trigger gamePieceToggleBtn;

    public static GamePiece requestedGamePiece = GamePiece.CONE;

    public RobotContainer() {
        autoChooser = new SendableChooser<Command>();
        autoChooser.addOption("None", Commands.print("No auto command selected"));
        autoChooser.addOption("Leave Zone", new LeaveZoneAutoCommand());
        autoChooser.addOption("Leave Zone and Charge", new LeaveZoneAndChargeAutoCommand());
        autoChooser.addOption("One Cone", new OneConeAutoCommand());
        autoChooser.addOption("One Cube", new OneCubeAutoCommand());
        autoChooser.setDefaultOption("Two Cube", new TwoCubeAutoCommand());
        Robot.MAIN_TAB.add(autoChooser);

        initCommands();
        configureBindings();

        pneumaticHub.enableCompressorDigital();
    }

    private void initCommands() {
        this.swerveDriveCommand = new SwerveDriveCommand();
    }

    private void configureBindings() {
        toggleGripperBtn = operatorController.leftBumper();
        toggleGripperBtn.onTrue(new GripperCommand(GripperAction.kToggle));

        highPosBtn = operatorController.a();
        highPosBtn.onTrue(new ArmMoveToPositionCommand(ArmPosition.kHigh));

        midPosBtn = operatorController.b();
        midPosBtn.onTrue(new ArmMoveToPositionCommand(ArmPosition.kMid));

        lowPosBtn = operatorController.x();
        lowPosBtn.onTrue(new ArmMoveToPositionCommand(ArmPosition.kLow));

        homePosBtn = operatorController.y();
        homePosBtn.onTrue(new ArmMoveToPositionCommand(ArmPosition.kHome));

        gamePieceToggleBtn = operatorController.rightBumper();
        gamePieceToggleBtn.onTrue(Commands.runOnce(() -> {
            requestedGamePiece = requestedGamePiece == GamePiece.CONE ? GamePiece.CUBE : GamePiece.CONE;
        }, lightingSubsystem));
    }

    public Command getTeleopDriveCommand() {
        return swerveDriveCommand;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public enum GamePiece {
        NONE(-1), CONE(0), CUBE(1);

        private int i;

        private GamePiece(int i){
            this.i = i;
        }

        public int valueOf(){
            return i;
        }
    }
}
