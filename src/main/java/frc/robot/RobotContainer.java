// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.GripperSubsystem;

public class RobotContainer {

  public static RobotContainer INSTANCE;
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final GripperSubsystem gripperSubsystem = new GripperSubsystem();

  CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_XBOX_CONTROLLER_ID);
  CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_XBOX_CONTROLLER_ID);

  Trigger toggleGripperBtn;
  Trigger highPosBtn;
  Trigger midPosBtn;
  Trigger lowPosBtn;
  Trigger homePosBtn;

  public RobotContainer() {
    configureBindings();
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
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public static RobotContainer getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new RobotContainer();
    }

    return INSTANCE;
  }
}
