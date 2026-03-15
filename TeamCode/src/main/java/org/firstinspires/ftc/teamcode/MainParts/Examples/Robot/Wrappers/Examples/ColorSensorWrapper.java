package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Wrappers.Examples;

import static com.qualcomm.hardware.ams.AMSColorSensor.AMS_TCS34725_ADDRESS;

import android.widget.RelativeLayout;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.Extenders2.UpdatableModule;

import java.lang.reflect.Field;

public class ColorSensorWrapper extends UpdatableModule {
    private AMSColorSensor amsColorSensor;
    private RevColorSensorV3 revColorSensorV3;
    private BroadcomColorSensor broadcomColorSensor;

    private RelativeLayout relativeLayout;

    public NormalizedRGBA rgba;
    private NormalizedColorSensor normalizedColorSensor;
    private DistanceSensor distanceSensor;

    public ColorSensorWrapper(String searchingDevice){
        super(searchingDevice);

        try {
            normalizedColorSensor = amsInit();

            if(normalizedColorSensor instanceof RevColorSensorV3){
                distanceSensor = ((DistanceSensor) normalizedColorSensor);
            }

            normalizedColorSensor.setGain(gain);
        }catch (Exception e){
            isInitialized = false;
        }

        tresholder = new double[4];
        sayInited();
    }

    private AMSColorSensor amsInit() throws Exception{

        AMSColorSensor amsColorSensor1 = hardwareMap.get(AdafruitI2cColorSensor.class, searchingDevice);
        AMSColorSensor.class.getDeclaredField("AMS_TCS34725_ADDRESS").setAccessible(true);

        AMSColorSensor.Parameters parameters = new AMSColorSensor.Parameters(AMS_TCS34725_ADDRESS, 0x4D);

        Field paramField = I2cDeviceSynchDeviceWithParameters.class.getDeclaredField("parameters");

        paramField.setAccessible(true);

        try {
            paramField.set(amsColorSensor1, parameters);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        amsColorSensor1.initialize(parameters);
        return amsColorSensor1;
    }
    private float gain = 15f;
    private double[] tresholder;
    private double r, g, b, a;
    private double distance;
    private int foundedColor;

    @Override
    protected void updateExt() {
        updateData();

        compareColor();
    }

    private void updateData(){
        rgba = normalizedColorSensor.getNormalizedColors();
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

        if(r > g && r > b && r > tresholder[0]) {
            redDetected = true;}
        else if(g > r && g > b && g > tresholder[1]) {
            greenDetected = true;}
        else if(b > r && b > g && b > tresholder[2]) {
            blueDetected = true;
        }else nothingDetected = true;

//        if(redDetected && distance > tresholder[4]) {
//            redDetected = false;}
//        else if(greenDetected && distance > tresholder[4]) {
//            greenDetected = false;}
//        else if(blueDetected && distance > tresholder[4]) {
//            blueDetected = false;
//        }else nothingDetected = true;

        if (!nothingDetected){
            foundedColor = redDetected || blueDetected ? 2 : 1;
        }else foundedColor = 0;
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
    public void showDataExt() {
        if(!isInitialized) sayBadInit();
        else {
            telemetry.addData("Founded color", getColorFromNumber(foundedColor));
            //Для отладки
            telemetry.addData("Colors", "R:%.3f G:%.3f B:%.3f A:%.3f", r, g, b, a);
            telemetry.addData("Distance", distance);
        }
    }

    public static class InnerBuilder extends Builder<ColorSensorWrapper> {
        @Override
        public InnerBuilder initialize(String searchingDevice) {
            wrapper = new ColorSensorWrapper(searchingDevice);
            return this;
        }

        @Override
        public InnerBuilder setFields(Double... args) {
            wrapper.tresholder[0] = args[0];
            wrapper.tresholder[1] = args[1];
            wrapper.tresholder[2] = args[2];
            wrapper.tresholder[3] = args[3];
            return this;
        }
    }
    public static class InnerCollector extends CollectorBuilder<ColorSensorWrapper>{
        @Override
        public CollectorBuilder add(ColorSensorWrapper wrapper) {
            wrappers.put(wrapper.searchingDevice, wrapper);
            return this;
        }

        @Override
        public void showData() {
            for (ColorSensorWrapper wrapper:wrappers.values()) {
                wrapper.showData();
            }
        }
    }
}