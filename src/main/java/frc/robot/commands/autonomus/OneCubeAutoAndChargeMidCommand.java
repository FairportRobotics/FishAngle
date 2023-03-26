package frc.robot.commands.autonomus;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmMoveToPositionCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.SwerveDrivePathCommand;
import frc.robot.commands.GripperCommand.GripperAction;
import frc.robot.subsystem.ArmSubsystem.ArmPosition;

public class OneCubeAutoAndChargeMidCommand extends SequentialCommandGroup {
    public OneCubeAutoAndChargeMidCommand() {
        addCommands(
                new GripperCommand(GripperAction.kClose),
                new ArmMoveToPositionCommand(ArmPosition.kHigh),
                new WaitCommand(0.5),
                new ParallelDeadlineGroup(
                        new SwerveDrivePathCommand("One_Cube_Place_Mid", true)),
                new ArmMoveToPositionCommand(ArmPosition.kHigh),
                new GripperCommand(GripperAction.kOpen),
                new WaitCommand(0.25),
                new ParallelDeadlineGroup(
                        new SwerveDrivePathCommand("One_Cube_Charge_Mid", new PathConstraints(2, 1.5),
                                false),
                        new SequentialCommandGroup(new WaitCommand(1),
                                new ArmMoveToPositionCommand(ArmPosition.kHome)
                                )),
                new AutoBalanceCommand());
    }
}
