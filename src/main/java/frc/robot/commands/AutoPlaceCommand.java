package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystem.swerve.SwerveDriveSubsystem;

public class AutoPlaceCommand extends SequentialCommandGroup {
    SwerveDriveSubsystem swerveDriveSubsystem;

    public AutoPlaceCommand(GridHeight height) {
        swerveDriveSubsystem = RobotContainer.swerveDriveSubsystem;

        if(!(swerveDriveSubsystem.getPose().getY() < 5.5 &&
                (swerveDriveSubsystem.getPose().getX() < 3.3
                || swerveDriveSubsystem.getPose().getX() > 13.2))) return;


        double distBetween = 0.55;
        for(int i = 0; i < 9; i++) {
            if(swerveDriveSubsystem.getPose().getY() > 0.521626 + (i * distBetween) - distBetween / 2 &&
                    swerveDriveSubsystem.getPose().getY() < 0.521626 + (i * distBetween) + distBetween / 2) {
                double yToGoTo = 0.521626 + (i * distBetween);
                double xToGoTo;
                double yawToGoTo;

                if(swerveDriveSubsystem.getPose().getX() < 3.3) {
                    xToGoTo = 2.5;
                    yawToGoTo = Math.PI;
                } else {
                    xToGoTo = 14;
                    yawToGoTo = 0;
                }
                addCommands(new GoToPostitionCommand(xToGoTo, yToGoTo, yawToGoTo));
                if(height == GridHeight.LOW) {
                    addCommands(
                            new ArmMoveToPositionCommand(ArmMoveToPositionCommand.ArmPosition.kLow),
                            new GripperCommand(GripperCommand.GripperAction.kClose)
                    );
                } else if(height == GridHeight.MID) {
                    addCommands(
                            new SwerveDrivePathCommand("One_Cube_Place", true),
                            new ArmMoveToPositionCommand(ArmMoveToPositionCommand.ArmPosition.kMid),
                            new GripperCommand(GripperCommand.GripperAction.kClose)
                    );
                } else if(height == GridHeight.HIGH) {
                    addCommands(
                            new SwerveDrivePathCommand("One_Cube_Place", true),
                            new ArmMoveToPositionCommand(ArmMoveToPositionCommand.ArmPosition.kHigh),
                            new GripperCommand(GripperCommand.GripperAction.kClose)
                    );
                }
                addCommands(new GoToPostitionCommand(1.92, yToGoTo, yawToGoTo),
                            new GripperCommand(GripperCommand.GripperAction.kOpen),
                            new GoToPostitionCommand(2.5, yToGoTo, yawToGoTo),
                            new ArmMoveToPositionCommand(ArmMoveToPositionCommand.ArmPosition.kHome)
                                    .alongWith(new GripperCommand(GripperCommand.GripperAction.kClose)));
                break;
            }
        }
    }

    public enum GridHeight {
        LOW,
        MID,
        HIGH
    }
}
