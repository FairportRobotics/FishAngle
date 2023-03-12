package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePiece;

public class GripperSubsystem extends SubsystemBase {

    private final int MIN_RED_CONE = 205;
    private final int MAX_RED_CONE = 265;
    private final int MIN_GREEN_CONE = 154;
    private final int MAX_GREEN_CONE = 214;
    private final int MIN_BLUE_CONE = 0;
    private final int MAX_BLUE_CONE = 30;
    private final int MIN_RED_CUBE = 123;
    private final int MAX_RED_CUBE = 183;
    private final int MIN_GREEN_CUBE = 32;
    private final int MAX_GREEN_CUBE = 92;
    private final int MIN_BLUE_CUBE = 151;
    private final int MAX_BLUE_CUBE = 211;

    DoubleSolenoid gripperSolenoid;
    CommandXboxController operatorController;
    PneumaticHub pneumaticHub;
    //TCS34725 colorSensor;

    private GamePiece foundGamePiece = GamePiece.NONE;

    public GripperSubsystem(CommandXboxController operatorController) {
        this.operatorController = operatorController;

        pneumaticHub = RobotContainer.pneumaticHub;
        gripperSolenoid = pneumaticHub.makeDoubleSolenoid(Constants.GRIPPER_SOLENOID_OPEN_ID,
                Constants.GRIPPER_SOLENOID_CLOSE_ID);
        gripperSolenoid.set(Value.kReverse);

        //colorSensor = new TCS34725();
    }

    @Override
    public void periodic() {

        //TCS34725_RGB currentColor = colorSensor.getRGB();

        //foundGamePiece = checkColors(currentColor.getR(), currentColor.getG(), currentColor.getB());

        // Logger
        Logger.getInstance().recordOutput("Gripper State", gripperSolenoid.get().toString());
        Logger.getInstance().recordOutput("Game Piece in Gripper", foundGamePiece.toString());
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

    public GamePiece getFoundGamePiece(){
        return foundGamePiece;
    }

    private GamePiece checkColors(int r, int g, int b) {
        if (r >= MIN_RED_CONE && r <= MAX_RED_CONE && g >= MIN_GREEN_CONE && g <= MAX_GREEN_CONE && b >= MIN_BLUE_CONE
            && b <= MAX_BLUE_CONE)
          return GamePiece.CONE;
        else if (r >= MIN_RED_CUBE && r <= MAX_RED_CUBE && g >= MIN_GREEN_CUBE && g <= MAX_GREEN_CUBE && b >= MIN_BLUE_CUBE
            && b <= MAX_BLUE_CUBE)
          return GamePiece.CUBE;
        else
          return GamePiece.NONE;
      }

}
