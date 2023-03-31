package frc.robot.subsystem;

//import org.apache.commons.lang3.ObjectUtils.Null;

import com.fairportrobotics.frc.poe.controllers.lighting.ArduinoLightingController;
import com.fairportrobotics.frc.poe.controllers.lighting.ArduinoLightingController.LightingCommand;
import com.fairportrobotics.frc.poe.controllers.lighting.ArduinoLightingController.LightingCommandBuilder;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightingSubsystem extends SubsystemBase {

    ArduinoLightingController lightingController;
    String currentColor = "";
    String[] queueCommands;



    public LightingCommand setCubeColor = new LightingCommandBuilder().fillAll("155060180").build();
    public LightingCommand setConeColor = new LightingCommandBuilder().fillAll("235185000").build();
    public LightingCommand off = new LightingCommandBuilder().fillAll("000000000").build();
    public LightingCommand fillRainbow = new LightingCommandBuilder().fillRainbow().build();
    public LightingCommand waiting = new LightingCommandBuilder().setSpecificLED("255255255", "0").setSpecificLED("255255255", "1").setSpecificLED("255255255", "2").setSpecificLED("255255255", "3").setSpecificLED("255255255", "4").shiftWrap().build();
    public LightingCommand shiftRainbow = new LightingCommandBuilder().fillRainbow().shiftWrap().build();


    public LightingSubsystem() {
        try {
            lightingController = new ArduinoLightingController(9600, Port.kUSB);
        } catch (Exception e) {
            e.printStackTrace();
            try {
                lightingController = new ArduinoLightingController(9600, Port.kUSB1);
            } catch (Exception e1) {
                e1.printStackTrace();
                try {
                    lightingController = new ArduinoLightingController(9600, Port.kUSB2);
                } catch (Exception e2) {
                    e2.printStackTrace();
                    lightingController = null;
                }
            }
        }  
    }
    
    public void executeColor(LightingCommand command){
        if(lightingController != null){
            lightingController.executeCommand(command);
        }
    }

}
