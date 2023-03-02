package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {

    CommandXboxController operatorController;

    TalonFX armFalcon;
    AnalogInput armPot;
    PIDController armPidController;
    SendableChooser<Boolean> isArmEnabled = new SendableChooser<Boolean>();
    double currentArmSpeed = 0.0;

    TalonFX wristFalcon;
    AnalogInput wristPot;
    PIDController wristPidController;
    SendableChooser<Boolean> isWristEnabled = new SendableChooser<Boolean>();
    double currentWristSpeed = 0.0;

    public ArmSubsystem(CommandXboxController operatorController) {
        this.operatorController = operatorController;

        armFalcon = new TalonFX(Constants.ARM_FALCON_ID);
        armFalcon.setInverted(false); // Flip this to true if it's driving the wrong way
        armPot = new AnalogInput(Constants.ARM_POT_ID);
        armPidController = new PIDController(1, 0, 0.2); // TODO: Tune
        armPidController.setTolerance(100);

        isArmEnabled.addOption("Enabled", true);
        isArmEnabled.setDefaultOption("Disabled", false);
        Robot.TESTING_TAB.add("Arm toggle", isArmEnabled);

        wristFalcon = new TalonFX(Constants.WRIST_FALCON_ID);
        wristFalcon.setInverted(false); // Flip this to true if it's driving the wrong way
        wristPot = new AnalogInput(Constants.WRIST_POT_ID);
        wristPidController = new PIDController(1, 0, 0.2); // TODO: Tune
        wristPidController.setTolerance(100);

        isWristEnabled.addOption("Enabled", true);
        isWristEnabled.setDefaultOption("Disabled", false);
    }

    @Override
    public void periodic() {
        setArmPosition(getArmSetpoint() + (operatorController.getLeftY() * 10));
        setWristPosition(getWristSetpoint() + (operatorController.getRightY() * 10));

        currentArmSpeed = armPidController.calculate(armPot.getValue());
        currentWristSpeed = wristPidController.calculate(wristPot.getValue());

        currentArmSpeed = Math.max(-100, Math.min(currentArmSpeed, 100));
        currentWristSpeed = Math.max(-100, Math.min(currentWristSpeed, 100));

        if (armPot.getValue() >= Constants.ARM_MAX_POT_VALUE) {
            currentArmSpeed = Math.min(currentArmSpeed, 0);
        } else if (armPot.getValue() <= Constants.ARM_MIN_POT_VALUE) {
            currentArmSpeed = Math.max(currentArmSpeed, 0);
        }

        if (wristPot.getValue() >= Constants.WRIST_MAX_POT_VALUE) {
            currentWristSpeed = Math.min(currentWristSpeed, 0);
        } else if (wristPot.getValue() <= Constants.WRIST_MIN_POT_VALUE) {
            currentWristSpeed = Math.max(currentWristSpeed, 0);
        }

        if (DriverStation.isTest() && !isArmEnabled.getSelected())
            currentArmSpeed = 0.0;
        armFalcon.set(ControlMode.Velocity, currentArmSpeed);

        if (DriverStation.isTest() && !isWristEnabled.getSelected())
            currentWristSpeed = 0.0;
        wristFalcon.set(ControlMode.Velocity, currentWristSpeed);

        Logger.getInstance().recordOutput("Arm setpoint", armPidController.getSetpoint());
        Logger.getInstance().recordOutput("Arm Position", armPot.getValue());
        Logger.getInstance().recordOutput("Arm Speed", currentArmSpeed);
        Logger.getInstance().recordOutput("Arm at position", armPidController.atSetpoint());

        Logger.getInstance().recordOutput("Wrist setpoint", wristPidController.getSetpoint());
        Logger.getInstance().recordOutput("Wrist Position", wristPot.getValue());
        Logger.getInstance().recordOutput("Wrist Speed", currentWristSpeed);
        Logger.getInstance().recordOutput("Wrist at position", wristPidController.atSetpoint());
    }

    public void setArmPosition(double pos) {
        armPidController.setSetpoint(pos);
    }

    public double getArmSetpoint() {
        return armPidController.getSetpoint();
    }

    public void setWristPosition(double pos) {
        wristPidController.setSetpoint(pos);
    }

    public double getWristSetpoint() {
        return wristPidController.getSetpoint();
    }

    public boolean isAtRequestedPosition() {
        return armPidController.atSetpoint() && wristPidController.atSetpoint();
    }

}
