package frc.robot.commands.autonomus;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmMoveToPositionCommand;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.SwerveDrivePathCommand;
import frc.robot.commands.ArmMoveToPositionCommand.ArmPosition;
import frc.robot.commands.GripperCommand.GripperAction;

public class OneCubeAutoAndLeaveCommand extends SequentialCommandGroup {
    public OneCubeAutoAndLeaveCommand() {
        addCommands(
                new GripperCommand(GripperAction.kClose),
                new ArmMoveToPositionCommand(ArmPosition.kMid),
                new ParallelDeadlineGroup(
                        new SwerveDrivePathCommand("One_Cube_Place", true)),
                new ArmMoveToPositionCommand(ArmPosition.kMid),
                new GripperCommand(GripperAction.kOpen),
                new WaitCommand(0.5),
                new ParallelDeadlineGroup(
                        new SwerveDrivePathCommand("One_Cube_Out", true),
                        new ArmMoveToPositionCommand(ArmPosition.kHome)),
                new GripperCommand(GripperAction.kClose));
    }
}
