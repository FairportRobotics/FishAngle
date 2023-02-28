package frc.robot.commands.autonomus;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveDrivePathCommand;

public class LeaveZoneAndChargeAutoCommand extends SequentialCommandGroup{
    public LeaveZoneAndChargeAutoCommand(){
        addCommands(
            new SwerveDrivePathCommand("Move_Out_Zone_Charge")
        );
    }
}
