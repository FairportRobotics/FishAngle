package frc.robot.commands.autonomus.old;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveDrivePathCommand;

public class LeaveZoneAutoCommand extends SequentialCommandGroup{
    public LeaveZoneAutoCommand(){
        addCommands(
            new SwerveDrivePathCommand("Move_Out_Zone", true)
        );
    }
}
