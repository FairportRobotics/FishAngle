package frc.robot.commands.autonomus.old;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmMoveToPositionCommand;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.SwerveDrivePathCommand;
import frc.robot.commands.GripperCommand.GripperAction;
import frc.robot.subsystem.ArmSubsystem.ArmPosition;

public class TwoCubeChargeAutoCommand extends SequentialCommandGroup {

    public TwoCubeChargeAutoCommand() {
        addCommands(
                new ParallelDeadlineGroup(
                        new SwerveDrivePathCommand("Two_Cube_Place_1", true),
                        new ArmMoveToPositionCommand(ArmPosition.kHome),
                        new GripperCommand(GripperAction.kClose)),
                new ArmMoveToPositionCommand(ArmPosition.kHigh),
                new GripperCommand(GripperAction.kOpen),
                new WaitCommand(1.0),
                new ParallelDeadlineGroup(
                        new SwerveDrivePathCommand("Two_Cube_Place_2", false),
                        new ArmMoveToPositionCommand(ArmPosition.kHome),
                        new GripperCommand(GripperAction.kClose)),
                new GripperCommand(GripperAction.kOpen),
                new ArmMoveToPositionCommand(ArmPosition.kLow),
                new WaitCommand(1.5),
                new GripperCommand(GripperAction.kClose),
                new WaitCommand(0.5),
                new ParallelDeadlineGroup(
                        new SwerveDrivePathCommand("Two_Cube_Place_3", false),
                        new ArmMoveToPositionCommand(ArmPosition.kHome)),
                new ArmMoveToPositionCommand(ArmPosition.kMid),
                new GripperCommand(GripperAction.kOpen),
                new WaitCommand(1.5),
                new ParallelDeadlineGroup(
                        new SwerveDrivePathCommand("Two_Cube_Place_Charge_4", false),
                        new ArmMoveToPositionCommand(ArmPosition.kHome),
                        new GripperCommand(GripperAction.kClose)));
    }
}
