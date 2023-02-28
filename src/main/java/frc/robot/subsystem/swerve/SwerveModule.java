package frc.robot.subsystem.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
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
import frc.robot.Robot;

public class SwerveModule {

    private final double ENCODER_TICKS_PER_METER;
    private final Translation2d MODULE_LOCATION;

    private WPI_TalonFX driveFalcon;
    private WPI_TalonFX swerveFalcon;
    private CANCoder encoder;

    private ProfiledPIDController swerveController;

    private SimpleMotorFeedforward swerveFeedForward;

    private String name;

    /**
     * Create a new swerve module class.
     */
    public SwerveModule(SwerveModuleBuilder builder) {
        driveFalcon = new WPI_TalonFX(builder.driveMotorId);

        swerveFalcon = new WPI_TalonFX(builder.steerMotorId);
        swerveFalcon.setInverted(true);
        swerveController = new ProfiledPIDController(builder.steerP, builder.steerI, builder.steerD,
                new TrapezoidProfile.Constraints(builder.maxAngleVel, builder.maxAngleAccel));
        swerveController.enableContinuousInput(0, 360);

        swerveFeedForward = new SimpleMotorFeedforward(0, 0);

        encoder = new CANCoder(builder.absoluteEncoderId);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        encoder.configMagnetOffset(builder.steerEncoderOffset);
        encoder.setPositionToAbsolute();

        name = builder.name;

        ENCODER_TICKS_PER_METER = builder.driveEncoderTicksPerMeter;
        MODULE_LOCATION = builder.moduleLocation;
    }

    /**
     * Get the angle of the module, taking the offset into account.
     * 
     * @return the direction the swerve module is pointing.
     */
    public double getCurrentSteerAngle() {
        return encoder.getPosition();
    }

    /**
     * Get the position of swerve module, consisting of a distance and a direction.
     * 
     * @return <code>SwerveModulePosition</code> of the current module.
     */
    public SwerveModulePosition getPosition() {
        double distance = driveFalcon.getSelectedSensorPosition() / ENCODER_TICKS_PER_METER;
        return new SwerveModulePosition(distance, Rotation2d.fromDegrees(getCurrentSteerAngle()));
    }

    /**
     * Get the current speed of the swerve module, in m/s.
     * 
     * @return encoder velocity converted to m/s.
     */
    public double getCurrentVelocity() {
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
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getCurrentSteerAngle()));
        double driveFalconVelocity = (state.speedMetersPerSecond * ENCODER_TICKS_PER_METER) * 10;
        double swerveFalconVoltage = swerveController.calculate(getCurrentSteerAngle(), state.angle.getDegrees())
                + swerveFeedForward.calculate(state.angle.getDegrees());

        driveFalcon.set(ControlMode.Velocity, driveFalconVelocity);
        swerveFalcon.setVoltage(swerveFalconVoltage);

        if (DriverStation.isTest()) {
            Robot.DEBUG_TAB.add(name + " Swerve Voltage", swerveFalconVoltage);
            Robot.DEBUG_TAB.add(name + " Drive Voltage", driveFalconVelocity);
            Robot.DEBUG_TAB.add(name + " Current Angle", getCurrentSteerAngle());
            Robot.DEBUG_TAB.add(name + " Current Velocity", getCurrentVelocity());
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

        private int steerMotorId;
        private double steerP = 0.0, steerI = 0.0, steerD = 0.0;

        private double maxAngleAccel = 0.0;
        private double maxAngleVel = 0.0;

        private int absoluteEncoderId = 0;
        private double steerEncoderOffset = 0.0;
        private double driveEncoderTicksPerMeter = 0.0;

        private String name = "";

        public SwerveModuleBuilder() {

        }

        public SwerveModuleBuilder setLocation(double x, double y) {
            moduleLocation = new Translation2d(x, y);
            return this;
        }

        public SwerveModuleBuilder setMotorIds(int driveMotorId, int steerMotorId) {
            this.driveMotorId = driveMotorId;
            this.steerMotorId = steerMotorId;
            return this;
        }

        public SwerveModuleBuilder setSteerPID(double p, double i, double d) {
            this.steerP = p;
            this.steerI = i;
            this.steerD = d;
            return this;
        }

        public SwerveModuleBuilder setSwerveProfile(double maxAngleAccel, double maxAngleVel) {
            this.maxAngleAccel = maxAngleAccel;
            this.maxAngleVel = maxAngleVel;
            return this;
        }

        public SwerveModuleBuilder setAbsoluteEncoderId(int encoderId) {
            this.absoluteEncoderId = encoderId;
            return this;
        }

        public SwerveModuleBuilder setSteerEncoderOffset(double offset) {
            this.steerEncoderOffset = offset;
            return this;
        }

        public SwerveModuleBuilder setDriveEncoderTicksPerMeter(double encoderTicksPerMeter) {
            this.driveEncoderTicksPerMeter = encoderTicksPerMeter;
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