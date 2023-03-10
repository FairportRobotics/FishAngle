package frc.robot.commands.autonomus;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveDrivePathCommand;

public class LeaveZoneAndChargeAutoCommand extends SequentialCommandGroup{
    public LeaveZoneAndChargeAutoCommand(){
        addCommands(
            new SwerveDrivePathCommand(PathPlanner.loadPath("Move_Out_Zone_Charge", new PathConstraints(5.0, 3.0)))
        );
    }
}
