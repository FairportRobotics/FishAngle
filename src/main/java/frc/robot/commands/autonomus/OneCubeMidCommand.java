package frc.robot.commands.autonomus;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmMoveToPositionCommand;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.SwerveDrivePathCommand;
import frc.robot.commands.GripperCommand.GripperAction;
import frc.robot.subsystem.ArmSubsystem.ArmPosition;

public class OneCubeMidCommand extends SequentialCommandGroup {
    public OneCubeMidCommand() {
        addCommands(
                new GripperCommand(GripperAction.kClose),
                new ArmMoveToPositionCommand(ArmPosition.kMid),
                new WaitCommand(0.75),
                new ParallelDeadlineGroup(
                        new SwerveDrivePathCommand("One_Cube_Place_Mid", true)),
                new ArmMoveToPositionCommand(ArmPosition.kMid),
                new GripperCommand(GripperAction.kOpen),
                new WaitCommand(0.5),
                new ParallelDeadlineGroup(
                        new SwerveDrivePathCommand("One_Cube_Reset_Mid", true),
                        new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new ArmMoveToPositionCommand(ArmPosition.kHome))),
                new GripperCommand(GripperAction.kClose));
    }
}
