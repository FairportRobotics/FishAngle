package frc.robot.subsystem;

//import org.apache.commons.lang3.ObjectUtils.Null;

import com.fairportrobotics.frc.poe.controllers.lighting.ArduinoLightingController;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightingSubsystem extends SubsystemBase {

    ArduinoLightingController lightingController;
    String currentColor = "";

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
    
    //Color rgb values have to be a multiple of 5 otherwise the arduino will never get the color correct
    public void setCubeColor() {
        if (lightingController != null) {
            this.lightingController.fillAll("155060180");
            currentColor = "155060180";
        }
    }

    public void setConeColor() {
        if (lightingController != null) {
            this.lightingController.fillAll("235185000");
            currentColor = "235185000";
        }
    }

    public void setColor(String color) {
        if (lightingController != null) {
            this.lightingController.fillAll(color);
            currentColor = color;
        }
    }

    public void rainbow() {
        if (lightingController != null) {
            this.lightingController.fillRainbow();
            currentColor = "rainbow";
        }
    }

    public void shiftWrap() {
        if (lightingController != null) {
            this.lightingController.shiftWrap();
        }
    }
    public String getColor() {
        return currentColor;
    }

    public ArduinoLightingController getLightingController(){
        return lightingController;
    }

    public void off(){
        if (lightingController != null)
            this.lightingController.fillAll("000000000");
    }

}
