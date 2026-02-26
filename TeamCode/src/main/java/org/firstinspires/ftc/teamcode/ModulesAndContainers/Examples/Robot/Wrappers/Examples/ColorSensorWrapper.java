package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples;

import static com.qualcomm.hardware.ams.AMSColorSensor.AMS_TCS34725_ADDRESS;

import android.widget.RelativeLayout;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.HardwareBuilder;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Extenders.DeviceUpdaterWrapper;

import java.lang.reflect.Field;
import java.util.HashMap;

public class ColorSensorWrapper extends DeviceUpdaterWrapper {
    private AMSColorSensor amsColorSensor;
    private RevColorSensorV3 revColorSensorV3;
    private BroadcomColorSensor broadcomColorSensor;

    private RelativeLayout relativeLayout;

    public NormalizedRGBA rgba;
    private NormalizedColorSensor normalizedColorSensor;
    private DistanceSensor distanceSensor;

    public ColorSensorWrapper(OpMode op, String deviceName){
        super(op);

        this.deviceName = deviceName;

        try {
            normalizedColorSensor = hardwareMap.get(NormalizedColorSensor.class, deviceName);

            if(normalizedColorSensor instanceof RevColorSensorV3){
                distanceSensor = ((DistanceSensor) normalizedColorSensor);
            }
        }catch (Exception e){
            isInitialized = false;
        }


        tresholder = new double[4];
        sayInited();
    }

    private void amsInit(){
        try {
            AMSColorSensor amsColorSensor1 = hardwareMap.get(AdafruitI2cColorSensor.class, deviceName);
            AMSColorSensor.class.getDeclaredField("AMS_TCS34725_ADDRESS").setAccessible(true);

            AMSColorSensor.Parameters parameters = new AMSColorSensor.Parameters(AMS_TCS34725_ADDRESS, 0x4D);

            Field paramField = I2cDeviceSynchDeviceWithParameters.class.getDeclaredField("parameters");

            paramField.setAccessible(true);

            try {
                paramField.set(amsColorSensor, parameters);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }

            amsColorSensor.initialize(parameters);
            amsColorSensor.setGain(gain);
        }catch (Exception e){
            isInitialized = false;
        }
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
        rgba = amsColorSensor.getNormalizedColors();
        r = rgba.red;
        g = rgba.green;
        b = rgba.blue;
        a = rgba.alpha;

        if(distanceSensor != null){
            distance = distanceSensor.getDistance(DistanceUnit.CM);
        }

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
        if(distanceSensor == null) return;
        //Этот случай может быть поидее только тогда когда дистанс умер или его нет
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
            telemetry.addLine("==="+ amsColorSensor.getDeviceName() +"===");
            telemetry.addData("Founded color",  getColorFromNumber(foundedColor));
            //Для отладки
            telemetry.addData("Colors", "R:%.3f G:%.3f B:%.3f A:%.3f", r, g, b, a);
            telemetry.addData("Distance", distance);
            telemetry.addLine();
        }
    }
    public static class Builder extends HardwareBuilder {
        private ColorSensorWrapper colorSensorWrapper;

        @Override
        public Builder initialize(OpMode op, String deviceName) {
            colorSensorWrapper = new ColorSensorWrapper(op, deviceName);
            return this;
        }
        public Builder setFields(double[] tresholder){
            colorSensorWrapper.tresholder = tresholder;
            return this;
        }
        public ColorSensorWrapper get(){
            return colorSensorWrapper;
        }
    }
}