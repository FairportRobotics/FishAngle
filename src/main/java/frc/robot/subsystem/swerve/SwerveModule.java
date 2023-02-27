package frc.robot.subsystem.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class SwerveModule {

    private final double ENCODER_TICKS_PER_METER;
    private final Translation2d MODULE_LOCATION;

    private WPI_TalonFX driveFalcon;
    private WPI_TalonFX swerveFalcon;
    private CANCoder encoder;

    private double offset;

    private PIDController driveController;
    private ProfiledPIDController swerveController;

    private SimpleMotorFeedforward driveFeedForward;
    private SimpleMotorFeedforward swerveFeedForward;

    private String name;

    /**
     * Create a new swerve module class.
     */
    public SwerveModule(SwerveModuleBuilder builder) {
        driveFalcon = new WPI_TalonFX(builder.driveMotorId);
        swerveFalcon = new WPI_TalonFX(builder.swerveMotorId);
        encoder = new CANCoder(builder.encoderId);

        this.offset = builder.encoderOffset;
        this.name = builder.name;

        this.ENCODER_TICKS_PER_METER = builder.encoderTicksPerMeter;
        this.MODULE_LOCATION = builder.moduleLocation;

        driveController = new PIDController(builder.driveP, builder.driveI, builder.driveD);
        swerveController = new ProfiledPIDController(builder.swerveP, builder.swerveI, builder.swerveD,
                new TrapezoidProfile.Constraints(builder.maxAngleVel, builder.maxAngleAccel));
        swerveController.enableContinuousInput(0, 360);
        encoder.setPositionToAbsolute();
        swerveFalcon.setInverted(true);
        driveFeedForward = new SimpleMotorFeedforward(0, 0);
        swerveFeedForward = new SimpleMotorFeedforward(0, 0);
    }

    /**
     * Get the angle of the module, taking the offset into account.
     * 
     * @return the direction the swerve module is pointing.
     */
    public double getAngle() {
        return encoder.getPosition() - offset;
    }

    /**
     * Get the position of swerve module, consisting of a distance and a direction.
     * 
     * @return <code>SwerveModulePosition</code> of the current module.
     */
    public SwerveModulePosition getPosition() {
        double distance = driveFalcon.getSelectedSensorPosition() / ENCODER_TICKS_PER_METER;
        SmartDashboard.putNumber(name + "Distance Meters", distance);
        return new SwerveModulePosition(distance, Rotation2d.fromDegrees(getAngle()));
    }

    /**
     * Get the current speed of the swerve module, in m/s.
     * 
     * @return encoder velocity converted to m/s.
     */
    public double getVelocity() {
        return driveFalcon.getSelectedSensorVelocity() * 10 / ENCODER_TICKS_PER_METER;
    }

    public Translation2d getModuleLocation() {
        return this.MODULE_LOCATION;
    }

    /**
     * Set the swerve module to a <code>SwerveModuleState</code>.
     * 
     * @param state target state for the swerve module to be in.
     */
    public void fromModuleState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAngle()));
        double driveFalconVoltage = driveController.calculate(getVelocity(), state.speedMetersPerSecond);
        double swerveFalconVoltage = swerveController.calculate(getAngle(), state.angle.getDegrees())
                + swerveFeedForward.calculate(swerveController.getSetpoint().velocity);

        driveFalcon.set(ControlMode.PercentOutput, driveFalconVoltage);
        swerveFalcon.setVoltage(swerveFalconVoltage);

        if (DriverStation.isTest()) {
            Robot.DEBUG_TAB.add(name + " Swerve Voltage", swerveFalconVoltage);
            Robot.DEBUG_TAB.add(name + " Drive Voltage", driveFalconVoltage);
            Robot.DEBUG_TAB.add(name + " Current Angle", getAngle());
            Robot.DEBUG_TAB.add(name + " Current Velocity", getVelocity());
            Robot.DEBUG_TAB.add(name + " Target Angle", state.angle.getDegrees());
            Robot.DEBUG_TAB.add(name + " Target Velocity",
                    state.speedMetersPerSecond);
            Robot.DEBUG_TAB.add(name + " Swerve Target Velocity",
                    (swerveController.getSetpoint().velocity));
        }

    }

    public static class SwerveModuleBuilder {

        private Translation2d moduleLocation;

        private int driveMotorId;
        private double driveP = 0.0, driveI = 0.0, driveD = 0.0;

        private int swerveMotorId;
        private double swerveP = 0.0, swerveI = 0.0, swerveD = 0.0;

        private double maxAngleAccel = 0.0;
        private double maxAngleVel = 0.0;

        private int encoderId = 0;
        private double encoderOffset = 0.0;
        private double encoderTicksPerMeter = 0.0;

        private String name = "";

        public SwerveModuleBuilder() {

        }

        public SwerveModuleBuilder setLocation(double x, double y) {
            moduleLocation = new Translation2d(x, y);
            return this;
        }

        public SwerveModuleBuilder setMotorIds(int driveMotorId, int swerveMotorId) {
            this.driveMotorId = driveMotorId;
            this.swerveMotorId = swerveMotorId;
            return this;
        }

        public SwerveModuleBuilder setDrivePID(double p, double i, double d) {
            this.driveP = p;
            this.driveI = i;
            this.driveD = d;
            return this;
        }

        public SwerveModuleBuilder setSwervePID(double p, double i, double d) {
            this.swerveP = p;
            this.swerveI = i;
            this.swerveD = d;
            return this;
        }

        public SwerveModuleBuilder setSwerveProfile(double maxAngleAccel, double maxAngleVel) {
            this.maxAngleAccel = maxAngleAccel;
            this.maxAngleVel = maxAngleVel;
            return this;
        }

        public SwerveModuleBuilder setEncoderId(int encoderId) {
            this.encoderId = encoderId;
            return this;
        }

        public SwerveModuleBuilder setEncoderOffset(double offset) {
            this.encoderOffset = offset;
            return this;
        }

        public SwerveModuleBuilder setEncoderTicksPerMeter(double encoderTicksPerMeter) {
            this.encoderTicksPerMeter = encoderTicksPerMeter;
            return this;
        }

        public SwerveModuleBuilder setName(String name) {
            this.name = name;
            return this;
        }

        public SwerveModule build() {
            return new SwerveModule(this);
        }

    }

}