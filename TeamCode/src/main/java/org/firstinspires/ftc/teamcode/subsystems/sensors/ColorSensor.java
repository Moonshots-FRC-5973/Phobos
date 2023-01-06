package org.firstinspires.ftc.teamcode.subsystems.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class ColorSensor {
    private NormalizedColorSensor colorSensor;
    private double multiplier;

    public ColorSensor(HardwareMap hardwareMap, double multiplier) {
        this.multiplier = multiplier;
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
    }

    public double getRed() {
        return colorSensor.getNormalizedColors().red;
    }
    public double getGreen() {
        return colorSensor.getNormalizedColors().green;
    }
    public double getBlue() {
        return colorSensor.getNormalizedColors().blue;
    }
    public boolean isRed() {
        return (getRed() > getGreen() && getRed() > getBlue());
    }
    public boolean isGreen() {
        return (getGreen() > getRed() && getGreen() > getBlue());
    }
    public boolean isBlue() {
        return (getBlue() > getRed() && getBlue() > getGreen());
    }
}