package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystem.swerve.SwerveDriveSubsystem;

public class SwerveDriveCommand extends CommandBase {

    CommandXboxController driverController;
    SwerveDriveSubsystem driveSubsystem;

    public SwerveDriveCommand() {
        this.driverController = RobotContainer.driverController;
        this.driveSubsystem = RobotContainer.swerveDriveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double x = driverController.getLeftX() * Math.abs(driverController.getLeftX());
        double y = driverController.getLeftY() * Math.abs(driverController.getLeftY());

        double rot = driverController.getRightX() * Math.abs(driverController.getRightX());

        driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, driveSubsystem.getRotation()));

    }

    @Override
    public boolean isFinished() {
        return false; // Never finish. This way our drive command is always running
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
