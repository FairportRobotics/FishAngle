package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystem.GripperSubsystem;

public class GripperCommand extends CommandBase{
    
    GripperSubsystem gripperSubsystem;
    GripperAction action;

    public GripperCommand(GripperAction action){
        gripperSubsystem = RobotContainer.gripperSubsystem;
        this.action = action;
        this.addRequirements(gripperSubsystem);
    }

    @Override
    public void initialize() {

        switch(action){
            case kOpen:
                gripperSubsystem.openGripper();
                break;
            case kClose:
                gripperSubsystem.closeGripper();
                break;
            case kToggle:
                gripperSubsystem.toggleGripper();
                break;
            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {
       return true; 
    }

    public enum GripperAction{
        kOpen,
        kClose,
        kToggle
    }

}
