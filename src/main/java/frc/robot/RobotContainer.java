// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmMoveToPositionCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.GripperCommand.GripperAction;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.GripperSubsystem;
import frc.robot.subsystem.LightingSubsystem;
import frc.robot.subsystem.ArmSubsystem.ArmPosition;
import frc.robot.subsystem.swerve.SwerveDriveSubsystem;

public class RobotContainer {

    public static final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_XBOX_CONTROLLER_ID);
    public static final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_XBOX_CONTROLLER_ID);

    public static final PneumaticHub pneumaticHub = new PneumaticHub(Constants.PH_CAN_ID);
    public static final Compressor COMPRESSOR = pneumaticHub.makeCompressor();

    public static final ArmSubsystem armSubsystem = new ArmSubsystem(operatorController);
    public static final GripperSubsystem gripperSubsystem = new GripperSubsystem(operatorController);
    public static final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(driverController);
    public static final LightingSubsystem lightingSubsystem = new LightingSubsystem();

    private SwerveDriveCommand swerveDriveCommand;
    private AutoBalanceCommand autoBalanceCommand;

    Trigger toggleGripperBtn;
    Trigger highPosBtn;
    Trigger stationPosBtn;
    Trigger midPosBtn;
    Trigger lowPosBtn;
    Trigger homePosBtn;
    Trigger foldedPosBtn;

    Trigger gamePieceToggleBtn;

    public static GamePiece requestedGamePiece = GamePiece.CONE;

    public RobotContainer() {

        initCommands();
        configureBindings();

        COMPRESSOR.enableDigital();
    }

    private void initCommands() {
        this.swerveDriveCommand = new SwerveDriveCommand();
        this.autoBalanceCommand = new AutoBalanceCommand();
    }

    private void configureBindings() {
        toggleGripperBtn = operatorController.leftBumper();
        toggleGripperBtn.onTrue(new GripperCommand(GripperAction.kToggle));

        highPosBtn = operatorController.y();
        highPosBtn.onTrue(new ArmMoveToPositionCommand(ArmPosition.kHigh));

        stationPosBtn = operatorController.povUp();
        stationPosBtn.onTrue(new ArmMoveToPositionCommand(ArmPosition.kStation));

        midPosBtn = operatorController.b();
        midPosBtn.onTrue(new ArmMoveToPositionCommand(ArmPosition.kMid));

        lowPosBtn = operatorController.a();
        lowPosBtn.onTrue(new ArmMoveToPositionCommand(ArmPosition.kLow));

        homePosBtn = operatorController.x();
        homePosBtn.onTrue(new ArmMoveToPositionCommand(ArmPosition.kHome));

        foldedPosBtn = operatorController.back();
        foldedPosBtn.onTrue(new ArmMoveToPositionCommand(ArmPosition.kFolded));

        gamePieceToggleBtn = operatorController.rightBumper();
        gamePieceToggleBtn.onTrue(Commands.runOnce(() -> {
            requestedGamePiece = requestedGamePiece == GamePiece.CONE ? GamePiece.CUBE : GamePiece.CONE;
            if (requestedGamePiece == GamePiece.CONE) {
                RobotContainer.lightingSubsystem.setConeColor();
            } else if (requestedGamePiece == GamePiece.CUBE) {
                RobotContainer.lightingSubsystem.setCubeColor();
            } else {
                RobotContainer.lightingSubsystem.off();
                System.out.println("WHY NO LIGHTS!");
            }
        }));

        driverController.a().onTrue(this.autoBalanceCommand);
    }

    public Command getTeleopDriveCommand() {
        return swerveDriveCommand;
    }

    public Command getAutoBalanceCommand(){
        return this.autoBalanceCommand;
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
