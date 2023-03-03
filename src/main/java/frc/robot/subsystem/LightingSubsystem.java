package frc.robot.subsystem;

import com.fairportrobotics.frc.poe.controllers.lighting.ArduinoLightingController;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightingSubsystem extends SubsystemBase {

    ArduinoLightingController lightingController;

    public LightingSubsystem() {
        lightingController = new ArduinoLightingController(9600, Port.kUSB);
    }
    
    //Color rgb values have to be a multiple of 5 otherwise the arduino will never get the color correct
    public void setCubeColor() {
        this.lightingController.fillAll("155060180");
    }

    public void setConeColor() {
        this.lightingController.fillAll("235185000");
    }

    public void rainbow() {
        this.lightingController.fillRainbow();
    }

    public void off(){
        this.lightingController.fillAll("000000000");
    }

}
