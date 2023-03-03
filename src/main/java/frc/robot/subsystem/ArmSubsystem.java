package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    CommandXboxController operatorController;

    TalonFX armFalcon;
    AnalogInput armPot;
    PIDController armPidController;
    double currentArmSpeed = 0.0;

    TalonFX wristFalcon;
    AnalogInput wristPot;
    PIDController wristPidController;
    double currentWristSpeed = 0.0;

    Mechanism2d armMechanism2d;
    MechanismLigament2d armLig;
    MechanismLigament2d wristLig;

    public ArmSubsystem(CommandXboxController operatorController) {
        this.operatorController = operatorController;

        armFalcon = new TalonFX(Constants.ARM_FALCON_ID);
        armFalcon.setInverted(false); // Flip this to true if it's driving the wrong way
        armPot = new AnalogInput(Constants.ARM_POT_ID);
        armPidController = new PIDController(.001, 0, 0.2); // TODO: Tune
        armPidController.setTolerance(100);

        wristFalcon = new TalonFX(Constants.WRIST_FALCON_ID);
        wristFalcon.setInverted(false); // Flip this to true if it's driving the wrong way
        wristPot = new AnalogInput(Constants.WRIST_POT_ID);
        wristPidController = new PIDController(1, 0, 0.2); // TODO: Tune
        wristPidController.setTolerance(100);

        armMechanism2d = new Mechanism2d(10, 10);
        MechanismRoot2d root = armMechanism2d.getRoot("Arm", 7, 5);

        armLig = root.append(new MechanismLigament2d("Arm", 3, 180, 10, new Color8Bit(Color.kBlue)));
        wristLig = armLig.append(new MechanismLigament2d("Wrist", 1, 180, 10, new Color8Bit(Color.kPurple)));
    }

    @Override
    public void periodic() {

        // TODO: Map Potentiometer angle to wrist and arm angles
        //wristLig.setAngle(wristPot.getValue());
        //armLig.setAngle(armPot.getValue());

        setArmPosition(getArmSetpoint() + (operatorController.getLeftY() * 10));
        setWristPosition(getWristSetpoint() + (operatorController.getRightY() * 10));

        currentArmSpeed = armPidController.calculate(armPot.getValue());
        currentWristSpeed = wristPidController.calculate(wristPot.getValue());

        currentArmSpeed = Math.max(-0.25, Math.min(currentArmSpeed, 0.25));
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

        wristFalcon.set(ControlMode.Velocity, currentWristSpeed);

        Logger.getInstance().recordOutput("Arm setpoint", armPidController.getSetpoint());
        Logger.getInstance().recordOutput("Arm Position", armPot.getValue());
        Logger.getInstance().recordOutput("Arm Speed", currentArmSpeed);
        Logger.getInstance().recordOutput("Arm at position", armPidController.atSetpoint());

        Logger.getInstance().recordOutput("Wrist setpoint", wristPidController.getSetpoint());
        Logger.getInstance().recordOutput("Wrist Position", wristPot.getValue());
        Logger.getInstance().recordOutput("Wrist Speed", currentWristSpeed);
        Logger.getInstance().recordOutput("Wrist at position", wristPidController.atSetpoint());

        Logger.getInstance().recordOutput("Arm Mechanism", armMechanism2d);
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
