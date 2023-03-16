package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {

    public static double WRIST_SETPOINT_TOLERANCE = 50.0;
    public static double ARM_SETPOINT_TOLERANCE = 50.0;

    CommandXboxController operatorController;

    WPI_TalonFX armFalcon;
    double armSetpoint = 0.0;
    AnalogInput armPot;
    PIDController armPidController;
    boolean armPidDone = true;

    WPI_TalonFX wristFalcon;
    double wristSetpoint = 0.0;

    Mechanism2d armMechanism2d;
    MechanismLigament2d armLig;
    MechanismLigament2d wristLig;

    PneumaticHub pneumaticHub;
    Solenoid brakeSolenoid;

    public ArmSubsystem(CommandXboxController operatorController) {
        this.operatorController = operatorController;

        armFalcon = new WPI_TalonFX(Constants.ARM_FALCON_ID);
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        armConfig.slot0.kP = 1.0;
        armConfig.slot0.kI = 0.0;
        armConfig.slot0.kD = 0.2;
        armConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        armConfig.peakOutputForward = 0.75;
        armConfig.peakOutputReverse = -0.75;
        armFalcon.configAllSettings(armConfig);
        armFalcon.setInverted(false); // Flip this to true if it's driving the wrong way
        armFalcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        armFalcon.setNeutralMode(NeutralMode.Brake);
        armPot = new AnalogInput(Constants.ARM_POT_ID);
        armPidController = new PIDController(0.010, 0.0001, 0.00001); // TODO: Tune
        armPidController.setTolerance(10);

        wristFalcon = new WPI_TalonFX(Constants.WRIST_FALCON_ID);
        TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        wristConfig.slot0.kP = 1.0;
        wristConfig.slot0.kI = 0.0;
        wristConfig.slot0.kD = 0.2;
        wristConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        wristConfig.peakOutputForward = 0.25;
        wristConfig.peakOutputReverse = -0.25;
        wristFalcon.configAllSettings(wristConfig);
        wristFalcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        wristFalcon.setInverted(true); // Flip this to true if it's driving the wrong way
        wristFalcon.setNeutralMode(NeutralMode.Brake);

        pneumaticHub = RobotContainer.pneumaticHub;
        brakeSolenoid = pneumaticHub.makeSolenoid(Constants.ARM_BRAKE_SOLENOID);

        armMechanism2d = new Mechanism2d(10, 10);
        MechanismRoot2d root = armMechanism2d.getRoot("Arm", 7, 5);

        armLig = root.append(new MechanismLigament2d("Arm", 3, 180, 10, new Color8Bit(Color.kBlue)));
        wristLig = armLig.append(new MechanismLigament2d("Wrist", 1, 180, 10, new Color8Bit(Color.kPurple)));
    }

    @Override
    public void periodic() {

        double currentArmSpeed = 0.0;
        double currentWristSpeed = 0.0;

        if (MathUtil.applyDeadband(operatorController.getLeftY(), 0.1) != 0.0) { // Manual control
            currentArmSpeed = -operatorController.getLeftY();

            currentArmSpeed = Math.max(-0.5, Math.min(currentArmSpeed, 0.5));

            if (armPot.getValue() >= Constants.ARM_MAX_ROM_VALUE) {
                currentArmSpeed = Math.min(currentArmSpeed, 0);
            } else if (armPot.getValue() <= Constants.ARM_MIN_ROM_VALUE) {
                currentArmSpeed = Math.max(currentArmSpeed, 0);
            }

            armFalcon.set(ControlMode.PercentOutput, currentArmSpeed);
            armPidController.setSetpoint(armPot.getValue());

            armSetpoint = armFalcon.getSelectedSensorPosition();
        } else { // Setpoint
            armFalcon.set(ControlMode.Position, armSetpoint);
        }

        if (MathUtil.applyDeadband(operatorController.getRightY(), 0.1) != 0.0) { // Manual control
            currentWristSpeed = operatorController.getRightY();
            currentWristSpeed = Math.max(-0.30, Math.min(currentWristSpeed, 0.30));

            if (wristFalcon.getSelectedSensorPosition() >= Constants.WRIST_MAX_ROM_VALUE) {
                currentWristSpeed = Math.min(currentWristSpeed, 0);
            } else if (wristFalcon.getSelectedSensorPosition() <= Constants.WRIST_MIN_ROM_VALUE) {
                currentWristSpeed = Math.max(currentWristSpeed, 0);
            }

            wristFalcon.set(ControlMode.PercentOutput, currentWristSpeed);
            wristSetpoint = wristFalcon.getSelectedSensorPosition(); // Set the setpoint to where we currently are. This
                                                                     // way when the operator stops manual control it
                                                                     // holds where it currently is
        } else { // Setpoint
            wristFalcon.set(ControlMode.Position, wristSetpoint);
        }

        Logger.getInstance().recordOutput("Arm setpoint", armSetpoint);
        Logger.getInstance().recordOutput("Arm Position", armFalcon.getSelectedSensorPosition());
        Logger.getInstance().recordOutput("Arm Speed", currentArmSpeed);
        Logger.getInstance().recordOutput("Arm at position", isArmAtSetpoint());

        // Logger.getInstance().recordOutput("Arm Brake State", brakeSolenoid.get());

        Logger.getInstance().recordOutput("Wrist setpoint", wristSetpoint);
        Logger.getInstance().recordOutput("Wrist Position", wristFalcon.getSelectedSensorPosition());
        Logger.getInstance().recordOutput("Wrist Speed", currentWristSpeed);
        Logger.getInstance().recordOutput("Wrist at position", isWristAtSetpoint());

        Logger.getInstance().recordOutput("Arm Mechanism", armMechanism2d);
    }

    public boolean isArmAtSetpoint(){
        return armFalcon.getClosedLoopError(0) <= ARM_SETPOINT_TOLERANCE;
    }

    public void setArmPosition(double pos) {
        pos = Math.max(Constants.ARM_MIN_ROM_VALUE, Math.min(pos, Constants.ARM_MAX_ROM_VALUE));

        armSetpoint = pos;
    }

    public double getArmSetpoint() {
        return armPidController.getSetpoint();
    }

    public boolean isWristAtSetpoint(){
        return wristFalcon.getClosedLoopError(0) <= WRIST_SETPOINT_TOLERANCE;
    }

    public void setWristPosition(double pos) {
        pos = Math.max(Constants.WRIST_MIN_ROM_VALUE, Math.min(pos, Constants.WRIST_MAX_ROM_VALUE));
        wristSetpoint = pos;
    }

    public boolean isAtRequestedPosition() {
        return armPidController.atSetpoint() && isWristAtSetpoint();
    }

}
