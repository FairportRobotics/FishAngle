package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{

    TalonFX armFalcon;
    AnalogInput armPot;
    PIDController armPidController;

    TalonFX wristFalcon;
    AnalogInput wristPot;
    PIDController wristPidController;

    public ArmSubsystem(){
        armFalcon = new TalonFX(Constants.ARM_FALCON_ID);
        armFalcon.setInverted(false); // Flip this to true if it's driving the wrong way
        armPot = new AnalogInput(Constants.ARM_POT_ID);
        armPidController = new PIDController(0, 0, 0); // TODO: Tune

        wristFalcon = new TalonFX(Constants.WRIST_FALCON_ID);
        wristFalcon.setInverted(false); // Flip this to true if it's driving the wrong way
        wristPot = new AnalogInput(Constants.WRIST_POT_ID);
        wristPidController = new PIDController(0, 0, 0); // TODO: Tune
    }

    @Override
    public void periodic() {

        double armSpeed = armPidController.calculate(armPot.getValue());
        double wristSpeed = wristPidController.calculate(wristPot.getValue());
        
        if(armPot.getValue() >= Constants.ARM_MAX_POT_VALUE){
            armSpeed = Math.min(armSpeed, 0);
        }else if(armPot.getValue() <= Constants.ARM_MIN_POT_VALUE){
            armSpeed = Math.max(armSpeed, 0);
        }

        if(wristPot.getValue() >= Constants.WRIST_MAX_POT_VALUE){
            wristSpeed = Math.min(wristSpeed, 0);
        }else if(wristPot.getValue() <= Constants.WRIST_MIN_POT_VALUE){
            wristSpeed = Math.max(wristSpeed, 0);
        }

        armFalcon.set(ControlMode.PercentOutput, armSpeed);
        wristFalcon.set(ControlMode.PercentOutput, wristSpeed);
    }

    public void setArmPosition(double pos){
        armPidController.setSetpoint(pos);
    }

    public void setWristPosition(double pos){
        wristPidController.setSetpoint(pos);
    }

}
