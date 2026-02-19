package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers;

import android.widget.RelativeLayout;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.HardwareBuilder;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;

import java.util.HashMap;

public class ColorSensorWrapper extends UpdatableModule {
    private NormalizedColorSensor colorSensor;
    private RelativeLayout relativeLayout;
    public NormalizedRGBA rgba;
    public ColorSensorWrapper(OpMode op, String deviceName){
        super(op);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, deviceName);

        tresholder = new double[4];

        colorSensor.setGain(gain);

        telemetry.addLine("ColorSensor Inited");
    }
    private float gain = 15f;
    private double[] tresholder;
    private double r, g, b, a;
    private double distance;
    private int foundedColor;

    @Override
    public void update(){
        updateData();

        compareColor();
    }
    private void updateData(){
        rgba = colorSensor.getNormalizedColors();
        r = rgba.red;
        g = rgba.green;
        b = rgba.blue;
        a = rgba.alpha;

        distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
    }

    private void compareColor(){
        boolean redDetected = false;
        boolean greenDetected = false;
        boolean blueDetected = false;
        boolean nothingDetected = false;

        //Смотрим какой цвет доминирует и накрыт ли датчик света
        compareColor(redDetected, greenDetected, blueDetected, nothingDetected);
        compareDistance(redDetected, greenDetected, blueDetected, nothingDetected);

        if (!nothingDetected){
            foundedColor = redDetected || blueDetected ? 2 : 1;
        }else foundedColor = 0;
    }
    private void compareColor(boolean redDetected, boolean greenDetected, boolean blueDetected, boolean nothingDetected){
        if(r > g && r > b && r > tresholder[0]) {
            redDetected = true;}
        else if(g > r && g > b && g > tresholder[1]) {
            greenDetected = true;}
        else if(b > r && b > g && b > tresholder[2]) {
            blueDetected = true;
        }else nothingDetected = true;
    }
    private void compareDistance(boolean redDetected, boolean greenDetected, boolean blueDetected, boolean nothingDetected){
        //Этот случай может быть поидее только тогда когда дистанс умер
        if(distance == 0) return;
        if(redDetected && distance > tresholder[4]) {
            redDetected = false;}
        else if(greenDetected && distance > tresholder[4]) {
            greenDetected = false;}
        else if(blueDetected && distance > tresholder[4]) {
            blueDetected = false;
        }else nothingDetected = true;
    }
    private String getColorFromNumber(double number){
        return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
    }
    public int getFoundedColor() {
        return foundedColor;
    }
    public void setTresholder(double[] tresholder){
        this.tresholder = tresholder;
    }
    @Override
    public void showData(){
        telemetry.addLine("==="+ colorSensor.getDeviceName() +"===");
        telemetry.addData("Founded color",  getColorFromNumber(foundedColor));
        //Для отладки
        telemetry.addData("Colors", "R:%.3f G:%.3f B:%.3f A:%.3f", r, g, b, a);
        telemetry.addData("Distance", distance);
        telemetry.addLine();
    }
    public static class Builder extends HardwareBuilder {
        private HashMap<String, ColorSensorWrapper> sensors = new HashMap<>();

        @Override
        public Builder initialize(OpMode op, String deviceName) {
            sensors.put(deviceName, new ColorSensorWrapper(op, deviceName));
            return this;
        }
        public Builder setFields(String deviceName, double[] tresholder){
            sensors.get(deviceName).tresholder = tresholder;;
            return this;
        }
        public void updateAll(){
            for (ColorSensorWrapper sensor: sensors.values()) {
                sensor.update();
            }
        }

        public int foundedColor(){
            int color = 0;
            for (ColorSensorWrapper sensor : sensors.values()) {
                color = sensor.getFoundedColor();
                //как только любой из колоров нашёл что то выходим
                if (color != 0) break;
            }
            return color;
        }
        public void showData(){
            for (ColorSensorWrapper sensor: sensors.values()) {
                sensor.showData();
            }
        }
    }
}