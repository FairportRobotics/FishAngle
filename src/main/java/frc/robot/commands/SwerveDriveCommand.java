package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystem.swerve.SwerveDriveSubsystem;

public class SwerveDriveCommand extends CommandBase{
    
    CommandXboxController driverController;
    SwerveDriveSubsystem driveSubsystem;

    public SwerveDriveCommand(CommandXboxController driverController, SwerveDriveSubsystem driveSubsystem){
        this.driverController = driverController;
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void execute() {
        
        double x = driverController.getLeftX() * Math.abs(driverController.getLeftX());
        double y = driverController.getLeftY() * Math.abs(driverController.getLeftY());

        double rot = driverController.getRightX() * Math.abs(driverController.getRightX());

        driveSubsystem.drive(y, x, rot, driveSubsystem.getYaw());
    }

    @Override
    public boolean isFinished() {
        return false; // Never finish. This way our drive command is always running
    }
}
