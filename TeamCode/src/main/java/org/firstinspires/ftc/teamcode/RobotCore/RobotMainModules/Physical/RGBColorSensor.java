package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
//SensorColor.java
public class RGBColorSensor implements Module {
    private final OpMode op;
    private RevColorSensorV3 colorSensor;
    private double red;
    private double blue;
    private double green;
    private double distance;

    // TODO реализовать методы для датчика
    public RGBColorSensor(OpMode op){
        this.op = op;
    }

    @Override
    public void init() {
        this.colorSensor = op.hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        this.red = colorSensor.getNormalizedColors().red;
        this.green = colorSensor.getNormalizedColors().green;
        this.blue = colorSensor.getNormalizedColors().blue;
        this.distance = colorSensor.getDistance(DistanceUnit.CM);
    }

    public RevColorSensorV3 getSensorColor() {
        return colorSensor;
    }

    public double getBlue() {
        return blue;
    }

    public double getGreen() {
        return green;
    }

    public double getRed() {
        return red;
    }
    public String getMainColor() {
        if ((red + blue) >= 0.007) {
            return "White";
        } else {
            if (red > green && red > blue) {
                return "Red";
            } else if (green > red && green > blue) {
                return "Green";
            } else if (blue > green && blue > red) {
                return "Blue";
            } else  {
                return "Yellow";}
            }
    }

    public double getDistance() {
        return distance;
    }
}
