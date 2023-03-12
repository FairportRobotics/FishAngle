package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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

    CommandXboxController operatorController;

    TalonFX armFalcon;
    AnalogInput armPot;
    PIDController armPidController;
    double currentArmSpeed = 0.0;

    TalonSRX wristFalcon;
    AnalogInput wristPot;
    PIDController wristPidController;
    double currentWristSpeed = 0.0;

    Mechanism2d armMechanism2d;
    MechanismLigament2d armLig;
    MechanismLigament2d wristLig;

    PneumaticHub pneumaticHub;
    Solenoid brakeSolenoid;

    public ArmSubsystem(CommandXboxController operatorController) {
        this.operatorController = operatorController;

        armFalcon = new TalonFX(Constants.ARM_FALCON_ID);
        armFalcon.setInverted(false); // Flip this to true if it's driving the wrong way
        armFalcon.setSelectedSensorPosition(0);
        armFalcon.setNeutralMode(NeutralMode.Brake);
        armPot = new AnalogInput(Constants.ARM_POT_ID);
        armPidController = new PIDController(0.010, 0.0001, 0.00001); // TODO: Tune
        armPidController.setTolerance(50);

        wristFalcon = new TalonSRX(Constants.WRIST_FALCON_ID);
        wristFalcon.setInverted(true); // Flip this to true if it's driving the wrong way
        wristPot = new AnalogInput(Constants.WRIST_POT_ID);
        wristPidController = new PIDController(0.0015, 0, 0); // TODO: Tune
        wristPidController.setTolerance(50);

        pneumaticHub = RobotContainer.pneumaticHub;
        brakeSolenoid = pneumaticHub.makeSolenoid(Constants.ARM_BRAKE_SOLENOID);

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

        setArmPosition(getArmSetpoint() - (MathUtil.applyDeadband(operatorController.getLeftY(), 0.1) * 10));

        currentArmSpeed = armPidController.calculate(armPot.getValue());
        currentWristSpeed = wristPidController.calculate(wristPot.getValue());

        currentArmSpeed = Math.max(-0.5, Math.min(currentArmSpeed, 0.5));
        currentWristSpeed = Math.max(-0.75, Math.min(currentWristSpeed, 0.75));

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

        if(armPidController.atSetpoint()){
            brakeSolenoid.set(true);
            currentArmSpeed = 0.0;
        }else{
            brakeSolenoid.set(false);
        }


        armFalcon.set(ControlMode.PercentOutput, currentArmSpeed);

        if (MathUtil.applyDeadband(operatorController.getLeftY(), 0.1) == 0.0)
        {
            wristFalcon.set(ControlMode.PercentOutput, currentWristSpeed);
        }
        else  // manual override
        {
            currentWristSpeed = operatorController.getRightY()*0.75;
            wristFalcon.set(ControlMode.PercentOutput, currentWristSpeed);
        }

        Logger.getInstance().recordOutput("Arm setpoint", armPidController.getSetpoint());
        Logger.getInstance().recordOutput("Arm Position", armPot.getValue());
        Logger.getInstance().recordOutput("Arm Speed", currentArmSpeed);
        Logger.getInstance().recordOutput("Arm at position", armPidController.atSetpoint());

        //Logger.getInstance().recordOutput("Arm Brake State", brakeSolenoid.get());

        Logger.getInstance().recordOutput("Wrist setpoint", wristPidController.getSetpoint());
        Logger.getInstance().recordOutput("Wrist Position", wristPot.getValue());
        Logger.getInstance().recordOutput("Wrist Speed", currentWristSpeed);
        Logger.getInstance().recordOutput("Wrist at position", wristPidController.atSetpoint());

        Logger.getInstance().recordOutput("Arm Mechanism", armMechanism2d);
    }

    public void setArmPosition(double pos) {
        pos = Math.max(Constants.ARM_MIN_POT_VALUE, Math.min(pos, Constants.ARM_MAX_POT_VALUE));
        armPidController.setSetpoint(pos);
    }

    public double getArmSetpoint() {
        return armPidController.getSetpoint();
    }

    public void setWristPosition(double pos) {
        
        pos = Math.max(Constants.WRIST_MIN_POT_VALUE, Math.min(pos, Constants.WRIST_MAX_POT_VALUE));

        wristPidController.setSetpoint(pos);
    }

    public double getWristSetpoint() {
        return wristPidController.getSetpoint();
    }

    public boolean isAtRequestedPosition() {
        return armPidController.atSetpoint() && wristPidController.atSetpoint();
    }

}
