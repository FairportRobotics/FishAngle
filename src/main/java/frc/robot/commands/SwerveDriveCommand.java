package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystem.swerve.SwerveDriveSubsystem;

public class SwerveDriveCommand extends CommandBase {

    CommandXboxController driverController;
    SwerveDriveSubsystem driveSubsystem;

    TrapezoidProfile forwardProfile;

    public SwerveDriveCommand() {
        this.driverController = RobotContainer.driverController;
        this.driveSubsystem = RobotContainer.swerveDriveSubsystem;
        addRequirements(driveSubsystem);

        forwardProfile = new TrapezoidProfile(new Constraints(2.5, 1.0), new State());
    }

    @Override
    public void execute() {
        double strafe = driverController.getLeftX() * Math.abs(driverController.getLeftX());
        double forward = driverController.getLeftY() * Math.abs(driverController.getLeftY());
        double rot = driverController.getRightX() * Math.abs(driverController.getRightX());

        if(driverController.leftTrigger().getAsBoolean()){
            strafe *= 0.3;
            forward *= 0.3;
            rot *= 0.3;
        }

        forward = -MathUtil.applyDeadband(forward, 0.01);
        strafe = -MathUtil.applyDeadband(strafe, 0.01);
        rot = -MathUtil.applyDeadband(rot, 0.01);

        driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(forward * 2.25, strafe * 2.25, rot * 4, driveSubsystem.getRotation()));

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
