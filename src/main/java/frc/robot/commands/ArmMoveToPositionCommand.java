package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.subsystem.ArmSubsystem;

public class ArmMoveToPositionCommand extends CommandBase {

    ArmPosition position;
    ArmSubsystem armSubsystem;
    GamePiece requestedGamePiece;

    public ArmMoveToPositionCommand(ArmPosition postition) {
        this.position = postition;
        armSubsystem = RobotContainer.armSubsystem;
        requestedGamePiece = RobotContainer.requestedGamePiece;
        this.addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        double armPos = 0.0;
        double wristPos = 0.0;

        switch (position) {
            case kFolded:
                armPos = Constants.ARM_FOLDED_POS_VAL;
                wristPos = Constants.WRIST_FOLDED_POS_VAL;
                break;
            case kHome:
                armPos = Constants.ARM_HOME_POS[requestedGamePiece.valueOf()];
                wristPos = Constants.WRIST_HOME_POS[requestedGamePiece.valueOf()];
                break;
            case kLow:
                armPos = Constants.ARM_LOW_POS[requestedGamePiece.valueOf()];
                wristPos = Constants.WRIST_LOW_POS[requestedGamePiece.valueOf()];
                break;
            case kMid:
                armPos = Constants.ARM_MID_POS[requestedGamePiece.valueOf()];
                wristPos = Constants.WRIST_MID_POS[requestedGamePiece.valueOf()];
                break;
            case kHigh:
                armPos = Constants.ARM_HIGH_POS[requestedGamePiece.valueOf()];
                wristPos = Constants.WRIST_HIGH_POS[requestedGamePiece.valueOf()];
                break;
            default:
                break;
        }

        armSubsystem.setArmPosition(armPos);
        armSubsystem.setWristPosition(wristPos);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtRequestedPosition();
    }

    public enum ArmPosition {
        kFolded,
        kHome,
        kLow,
        kMid,
        kHigh
    }

}
