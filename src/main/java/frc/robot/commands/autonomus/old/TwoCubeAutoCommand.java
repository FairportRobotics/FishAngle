package frc.robot.commands.autonomus.old;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmMoveToPositionCommand;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.SwerveDrivePathCommand;
import frc.robot.commands.GripperCommand.GripperAction;
import frc.robot.subsystem.ArmSubsystem.ArmPosition;

public class TwoCubeAutoCommand extends SequentialCommandGroup{
    
    public TwoCubeAutoCommand(){
        addCommands(
            new ParallelDeadlineGroup(
                new SwerveDrivePathCommand("Two_Cube_Place_1", true),
                new ArmMoveToPositionCommand(ArmPosition.kHigh),
                new GripperCommand(GripperAction.kClose)
            ),
            new ArmMoveToPositionCommand(ArmPosition.kHigh),
            new WaitCommand(0.5),
            new GripperCommand(GripperAction.kOpen),
            new ParallelDeadlineGroup(
                new SwerveDrivePathCommand("Two_Cube_Place_2", false),
                new SequentialCommandGroup(
                    new WaitCommand(0.2),
                    new ArmMoveToPositionCommand(ArmPosition.kHome),
                    new GripperCommand(GripperAction.kClose)
                )
            ),
            new ParallelCommandGroup(
                new ArmMoveToPositionCommand(ArmPosition.kLow),
                new GripperCommand(GripperAction.kOpen)
            ),
            new WaitCommand(0.1),
            new GripperCommand(GripperAction.kClose),
            new ParallelDeadlineGroup(
                new SwerveDrivePathCommand("Two_Cube_Place_3", false),
                new SequentialCommandGroup(
                new ArmMoveToPositionCommand(ArmPosition.kHome),
                new WaitCommand(0.5),
                new ArmMoveToPositionCommand(ArmPosition.kMid)
                )
            ),
            new ArmMoveToPositionCommand(ArmPosition.kMid),
            new GripperCommand(GripperAction.kOpen),
            new WaitCommand(0.5),
            new ParallelDeadlineGroup(
                new SwerveDrivePathCommand("Two_Cube_Place_4", false),
                new ArmMoveToPositionCommand(ArmPosition.kHome),
                new GripperCommand(GripperAction.kClose)
            )
        );
    }
}
