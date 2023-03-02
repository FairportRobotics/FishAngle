package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystem.swerve.SwerveDriveSubsystem;

public class SetOffsetsPrintCommand extends CommandBase{
    
    SwerveDriveSubsystem swerveDriveSubsystem;

    public SetOffsetsPrintCommand(){
        swerveDriveSubsystem = RobotContainer.swerveDriveSubsystem;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        swerveDriveSubsystem.printOffsets();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
