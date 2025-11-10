package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Module;

public class ColorSensor extends Module {
    public ColorSensor(OpMode op){
        super(op.telemetry);

        colorSensor = op.hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        colorSensor.setGain(gain);

        relativeLayoutId = op.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", op.hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) op.hardwareMap.appContext).findViewById(relativeLayoutId);

        telemetry.addLine("ColorSensor Inited");
    }
    private final int relativeLayoutId;
    private final NormalizedColorSensor colorSensor;
    private final View relativeLayout;
    private float gain = 3f;
    private final float[] hsvValues = new float[3];
    public NormalizedRGBA colors;
    public float red;
    public float blue;
    public float green;
    public float alpha;
    public int currentArtifact;
    public int lastSeenArtifact;
    public double curDistance = 0;
    public void update(){
        colors = colorSensor.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);

//                relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));

        red = colors.red;
        blue = colors.blue;
        green = colors.green;
        alpha = colors.alpha;

        currentArtifact = getDominantColor();

        if(currentArtifact != 0) currentArtifact = lastSeenArtifact;

        curDistance = getDistance();
    }
    public int getDominantColor(){
        if(red > blue && red > green && red > 0.02) {

            return 2;}
        if(blue > red && blue > green && blue > 0.02) {

            return 2;}
        if(green > red && green > blue && green > 0.02) {

            return 1;}

        return 0;
    }
    public double getDistance(){
        return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
    }
    public String getColorFromNumber(int number){
        return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
    }
    public void showData(){
        telemetry.addLine("Color-sensor data")
                .addData("Current see color", "%s", getColorFromNumber(currentArtifact))
                .addData("Last seen color", "%s", getColorFromNumber(lastSeenArtifact))
                .addData("Distance in CM", "%.1f", getDistance());
        telemetry.addLine("Values from sensor")
                .addData("Red", "%.3f", red)
                .addData("Green", "%.3f", green)
                .addData("Blue", "%.3f", blue);
        telemetry.addLine();

    }

}