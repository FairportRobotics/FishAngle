package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystem.swerve.SwerveDriveSubsystem;

public class AutoBalanceCommand extends CommandBase {

    private static final float THRESHOLD = 5.0f; // Number in degrees

    SwerveDriveSubsystem swerveDriveSubsystem;
    AHRS gyro;
    ChassisSpeeds chassisSpeeds;

    float lastAccelX = 0.0f;
    float lastAccelY = 0.0f;
    float lastSpeedX = 0.0f;
    float lastSpeedY = 0.0f;

    float speedX = 0.1f;
    float speedY = 0.1f;

    public AutoBalanceCommand() {
        swerveDriveSubsystem = RobotContainer.swerveDriveSubsystem;
        gyro = swerveDriveSubsystem.getGyro();
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        swerveDriveSubsystem.drive(chassisSpeeds);
    }

    @Override
    public void execute() {

        float pitch = gyro.getPitch(); // Around the X axis aka, drive forward/back to fix
        float roll = gyro.getRoll(); // Around the Y axis aka, drive left/right to fix

        chassisSpeeds.vyMetersPerSecond = Math.signum(pitch) * -speedX; // Get the angle direction and multiply it by
                                                                        // how
                                                                        // fast we want to go.
                                                                        // "Why the negative?" you might ask. That's
                                                                        // because
                                                                        // if the angle is negative, we want to drive
                                                                        // positive direction and vice versa
        chassisSpeeds.vxMetersPerSecond = Math.signum(roll) * -speedY;

        swerveDriveSubsystem.drive(chassisSpeeds);

        // Get acceleration values after moving.
        lastAccelX = gyro.getWorldLinearAccelX();
        lastAccelY = gyro.getWorldLinearAccelY();
    }

    @Override
    public boolean isFinished() {
        boolean pitchGood = Math.abs(gyro.getPitch()) <= THRESHOLD;
        boolean rollGood = Math.abs(gyro.getRoll()) <= THRESHOLD;

        return pitchGood && rollGood;
    }

    @Override
    public void end(boolean interrupted) {

        if (interrupted) {
            chassisSpeeds.omegaRadiansPerSecond = 0;
            chassisSpeeds.vxMetersPerSecond = 0;
            chassisSpeeds.vyMetersPerSecond = 0;
        } else {
            chassisSpeeds.vxMetersPerSecond = 0;
            chassisSpeeds.vyMetersPerSecond = 0;
            chassisSpeeds.omegaRadiansPerSecond = 0.0001; // Turn the wheels 45 to rotate. Hopefully locking the wheels
        }

        swerveDriveSubsystem.drive(chassisSpeeds);
    }

}
