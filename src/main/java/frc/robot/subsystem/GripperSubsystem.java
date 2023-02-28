package frc.robot.subsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;

public class GripperSubsystem extends SubsystemBase {

    DoubleSolenoid gripperSolenoid;
    PneumaticHub pneumaticHub;
    SendableChooser<Boolean> isGripperEnabled = new SendableChooser<Boolean>();
    CommandXboxController operatorController;

    public GripperSubsystem(CommandXboxController operatorController) {
        this.operatorController = operatorController;

        pneumaticHub = new PneumaticHub();
        pneumaticHub.enableCompressorDigital(); // Start the compressor

        gripperSolenoid = pneumaticHub.makeDoubleSolenoid(Constants.GRIPPER_SOLENOID_OPEN_ID,
                Constants.GRIPPER_SOLENOID_CLOSE_ID);
        gripperSolenoid.set(Value.kReverse);

        isGripperEnabled.setDefaultOption("Disabled", false);
        isGripperEnabled.addOption("Enabled", true);
        Robot.TESTING_TAB.add("Gripper toggle", isGripperEnabled);
    }

    @Override
    public void periodic() {
    }

    public void toggleGripper() {
        if (DriverStation.isTest() && !isGripperEnabled.getSelected())
            return;
        if (gripperSolenoid.get() == Value.kReverse)
            gripperSolenoid.set(Value.kForward);
        else if (gripperSolenoid.get() == Value.kForward)
            gripperSolenoid.set(Value.kReverse);
    }

    public void openGripper() {
        if (DriverStation.isTest() && !isGripperEnabled.getSelected())
            return;
        gripperSolenoid.set(Value.kForward);
    }

    public void closeGripper() {
        if (DriverStation.isTest() && !isGripperEnabled.getSelected())
            return;
        gripperSolenoid.set(Value.kReverse);
    }

}
