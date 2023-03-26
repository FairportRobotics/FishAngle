package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystem.swerve.SwerveDriveSubsystem;

public class GoToPostitionCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;
    private final Pose2d targetPose;
    private final double maxMovementVelocity;
    private final double maxRotationVelocity;

    public GoToPostitionCommand(double x, double y, double yaw) {
        driveSubsystem = RobotContainer.swerveDriveSubsystem;
        targetPose = new Pose2d(x, y, new Rotation2d(yaw));
        maxMovementVelocity = 1.125;
        maxRotationVelocity = Math.PI;
        addRequirements(driveSubsystem);
    }

    public GoToPostitionCommand(double x, double y, double yaw, double maxMovementVelocity, double maxRotationVelocity) {
        driveSubsystem = RobotContainer.swerveDriveSubsystem;
        targetPose = new Pose2d(x, y, new Rotation2d(yaw));
        this.maxMovementVelocity = maxMovementVelocity;
        this.maxRotationVelocity = maxRotationVelocity;
        addRequirements(driveSubsystem);
    }

    /*
    * The following command is really complicated, so I will explain it here
    *
    * We start by seeing which is higher, the x or y distance, and we will set that to my chosen max velocity, 1.125
    * Then we will set the other velocity to the ratio of the distance to the max distance so that it arrives at the same time
    * if the rotation is greater than the distance, we will scale down the velocity so that it is rotated when it reaches the target
    * if the rotation is less than the distance, we will scale down the rotation so that it is rotated when it reaches the target
    * then we drive (:
     */
    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();
        double xDist = Math.abs(currentPose.getX() - targetPose.getX());
        double yDist = Math.abs(currentPose.getY() - targetPose.getY());
        double rotationDist = getDistBetweenRadian(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        //Set larger distance to 1.125 m/s and lower distance to arrive at same time, rotation is done so that it should be roughly the same time with a max of 3 rad/s
        double xVelocity;
        double yVelocity;
        double rotationVelocity = maxRotationVelocity;

        if(xDist == 0 && yDist == 0 && rotationDist == 0) { return; }

        if(rotationDist / rotationVelocity > Math.max(xDist, yDist) / maxMovementVelocity) {
            xVelocity = (targetPose.getX() - currentPose.getX()) / (rotationDist / rotationVelocity);
            yVelocity = (targetPose.getY() - currentPose.getY()) / (rotationDist / rotationVelocity);
        } else if(xDist >= yDist) {
            xVelocity = currentPose.getX() - targetPose.getX() < 0 ? maxMovementVelocity : -maxMovementVelocity;
            yVelocity = (currentPose.getY() - targetPose.getY()) / (currentPose.getX() - targetPose.getX()) * xVelocity;

            //if the x/y wouldn't finish in time, scale down the rotation
            if(rotationDist / rotationVelocity < Math.abs(xDist / xVelocity)) {
                rotationVelocity *=  (rotationDist / rotationVelocity) / Math.abs(xDist / xVelocity);
            }

        } else {
            yVelocity = currentPose.getY() - targetPose.getY() < 0 ? maxMovementVelocity : -maxMovementVelocity;
            xVelocity = (currentPose.getX() - targetPose.getX()) / (currentPose.getY() - targetPose.getY()) * yVelocity;

            //if the x/y wouldn't finish in time, scale down the rotation
            if(rotationDist / rotationVelocity < Math.abs(yDist / yVelocity)) {
                rotationVelocity *=  (rotationDist / rotationVelocity) / Math.abs(yDist / yVelocity);
            }
        }

        if ((targetPose.getRotation().getDegrees() - driveSubsystem.getPose().getRotation().getDegrees() + 540) % 360 - 180 > 0) {
            rotationVelocity *= -1;
        }


        driveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationVelocity, currentPose.getRotation()));
    }

    private double getDistBetweenRadian(double rad1, double rad2) {
        //get closest distance between two radian values
        double dist = Math.abs(rad1 - rad2);
        if(dist > Math.PI) {
            dist = 2 * Math.PI - dist;
        }
        return Math.abs(dist);
    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.05 && getDistBetweenRadian(targetPose.getRotation().getRadians(), driveSubsystem.getPose().getRotation().getRadians()) < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    }

}
