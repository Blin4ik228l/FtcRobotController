package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.Colors;

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

    public Colors getMainColor() {
        if (red == blue && red == green) {
            return Colors.WHITE;
        } else if (red + blue + green == 0) {
            return Colors.BLACK;
        } else if (red > green && red > blue) {
                return Colors.RED;
        } else if (green > red && green > blue) {
                return Colors.GREEN;
        } else if (blue > green && blue > red) {
                return Colors.BLUE;
        } else  {
            return Colors.YELLOW;
        }

    }

    public double getDistance() {
        return distance;
    }

    public void update(){
        distance = colorSensor.getDistance(DistanceUnit.CM);
        red = colorSensor.getNormalizedColors().red;
        green = colorSensor.getNormalizedColors().green;
        blue = colorSensor.getNormalizedColors().blue;
    }
}
