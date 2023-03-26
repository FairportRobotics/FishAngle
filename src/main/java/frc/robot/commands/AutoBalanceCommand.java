package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystem.swerve.SwerveDriveSubsystem;

public class AutoBalanceCommand extends CommandBase {

    private static final float THRESHOLD = 6.0f; // Number in degrees

    SwerveDriveSubsystem swerveDriveSubsystem;
    AHRS gyro;

    float lastAccelX = 0.0f;
    float lastAccelY = 0.0f;
    float lastSpeedX = 0.0f;
    float lastSpeedY = 0.0f;

    float speedX = 0.45f;
    float speedY = 0.45f;

    double waitTime = 0;

    public AutoBalanceCommand() {
        swerveDriveSubsystem = RobotContainer.swerveDriveSubsystem;
        gyro = swerveDriveSubsystem.getGyro();
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        Logger.getInstance().recordOutput("AutoBalance", true);
    }

    @Override
    public void execute() {

        float pitch = gyro.getRoll(); // Around the X axis aka, drive forward/back to fix
        float roll = gyro.getPitch(); // Around the Y axis aka, drive left/right to fix

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

        if (Math.abs(pitch) >= THRESHOLD) {
            chassisSpeeds.vxMetersPerSecond = Math.signum(pitch) * speedY; // Get the angle direction and multiply it
                                                                           // by
            // how
            // fast we want to go.
        }

        if (Math.abs(roll) >= THRESHOLD) {
            chassisSpeeds.vyMetersPerSecond = Math.signum(roll) * speedX;
        }

        swerveDriveSubsystem.drive(chassisSpeeds);

        Logger.getInstance().recordOutput("AutoBalance X", chassisSpeeds.vxMetersPerSecond);
        Logger.getInstance().recordOutput("AutoBalance Y", chassisSpeeds.vyMetersPerSecond);

        // Get acceleration values after moving.
        lastAccelX = gyro.getWorldLinearAccelX();
        lastAccelY = gyro.getWorldLinearAccelY();
    }

    @Override
    public boolean isFinished() {
        boolean pitchGood = Math.abs(gyro.getRoll()) <= THRESHOLD;
        boolean rollGood = Math.abs(gyro.getPitch()) <= THRESHOLD;

        // if(pitchGood && rollGood && waitTime == 0){
        //     waitTime = Timer.getFPGATimestamp();
        //     return false;
        // } else if(pitchGood && rollGood && waitTime != 0){
        //     return Timer.getFPGATimestamp() - waitTime >= 0.25;
        // } else if(waitTime != 0 && (!rollGood || !pitchGood)){
        //     waitTime = 0;
        //     return false;
        // }

        return pitchGood && rollGood;
    }

    @Override
    public void end(boolean interrupted) {

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

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
        Logger.getInstance().recordOutput("AutoBalance", false);
        RobotContainer.lightingSubsystem.shiftWrap();
    }

}
