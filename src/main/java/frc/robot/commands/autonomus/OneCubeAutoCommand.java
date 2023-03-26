package frc.robot.commands.autonomus;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmMoveToPositionCommand;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.SwerveDrivePathCommand;
import frc.robot.commands.GripperCommand.GripperAction;
import frc.robot.subsystem.ArmSubsystem.ArmPosition;

public class OneCubeAutoCommand extends SequentialCommandGroup {
    public OneCubeAutoCommand() {
        addCommands(
                new GripperCommand(GripperAction.kClose),
                new ArmMoveToPositionCommand(ArmPosition.kHigh),
                new ParallelDeadlineGroup(
                        new SwerveDrivePathCommand("One_Cube_Place", true)),
                new ArmMoveToPositionCommand(ArmPosition.kHigh),
                new GripperCommand(GripperAction.kOpen));
    }
}
