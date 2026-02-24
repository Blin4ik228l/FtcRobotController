package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers;

import static com.qualcomm.hardware.ams.AMSColorSensor.AMS_TCS34725_ADDRESS;

import android.widget.RelativeLayout;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.HardwareBuilder;

import java.lang.reflect.Field;
import java.util.HashMap;

public class ColorSensorWrapper extends DeviceUpdaterWrapper {
//    private NormalizedColorSensor colorSensor;
    private AdafruitI2cColorSensor colorSensor;
    private RelativeLayout relativeLayout;
    public NormalizedRGBA rgba;
    public ColorSensorWrapper(OpMode op, String deviceName){
        super(op);

        this.deviceName = deviceName;
        try {
            colorSensor = hardwareMap.get(AdafruitI2cColorSensor.class, deviceName);
            AMSColorSensor.class.getDeclaredField("AMS_TCS34725_ADDRESS").setAccessible(true);

            AMSColorSensor.Parameters parameters = new AMSColorSensor.Parameters(AMS_TCS34725_ADDRESS, 0x4D);

            Field paramField = I2cDeviceSynchDeviceWithParameters.class.getDeclaredField("parameters");

            paramField.setAccessible(true);

            try {
                paramField.set(colorSensor, parameters);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }

            colorSensor.initialize();
            colorSensor.setGain(gain);
        }catch (Exception e){
            isInitialized = false;
        }

        tresholder = new double[4];
        sayInited();
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
        if (!isInitialized) return;
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
        if(!isInitialized) telemetry.addLine("color" + " " + deviceName + "Not Found/Attached");
        else {
            telemetry.addLine("==="+ colorSensor.getDeviceName() +"===");
            telemetry.addData("Founded color",  getColorFromNumber(foundedColor));
            //Для отладки
            telemetry.addData("Colors", "R:%.3f G:%.3f B:%.3f A:%.3f", r, g, b, a);
            telemetry.addData("Distance", distance);
            telemetry.addLine();
        }
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
        public boolean isInited(){
            boolean isInited = true;
            for (ColorSensorWrapper sensor : sensors.values()) {
                isInited &= sensor.isInitialized;
            }
            return isInited;        }
        public void showData(){
            for (ColorSensorWrapper sensor: sensors.values()) {
                sensor.showData();
            }
        }
    }
}