package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {

    DoubleSolenoid gripperSolenoid;
    PneumaticHub pneumaticHub;
    CommandXboxController operatorController;

    public GripperSubsystem(CommandXboxController operatorController) {
        this.operatorController = operatorController;

        pneumaticHub = new PneumaticHub();
        pneumaticHub.enableCompressorDigital(); // Start the compressor

        gripperSolenoid = pneumaticHub.makeDoubleSolenoid(Constants.GRIPPER_SOLENOID_OPEN_ID,
                Constants.GRIPPER_SOLENOID_CLOSE_ID);
        gripperSolenoid.set(Value.kReverse);
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("Gripper State", gripperSolenoid.get().toString());
    }

    public void toggleGripper() {
        if (gripperSolenoid.get() == Value.kReverse)
            gripperSolenoid.set(Value.kForward);
        else if (gripperSolenoid.get() == Value.kForward)
            gripperSolenoid.set(Value.kReverse);
    }

    public void openGripper() {
        gripperSolenoid.set(Value.kForward);
    }

    public void closeGripper() {
        gripperSolenoid.set(Value.kReverse);
    }

}
