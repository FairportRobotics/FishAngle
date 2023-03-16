package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
        double strafe = driverController.getLeftX() * Math.abs(driverController.getLeftX());
        double forward = driverController.getLeftY() * Math.abs(driverController.getLeftY());
        double rot = driverController.getRightX() * Math.abs(driverController.getRightX());

        forward = -MathUtil.applyDeadband(forward, 0.05);
        strafe = -MathUtil.applyDeadband(strafe, 0.05);
        rot = MathUtil.applyDeadband(rot, 0.05);

        driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(forward * 2, strafe * 2, rot * 6, driveSubsystem.getRotation()));

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
