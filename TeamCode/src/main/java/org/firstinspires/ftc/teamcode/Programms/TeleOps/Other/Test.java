package org.firstinspires.ftc.teamcode.Programms.TeleOps.Other;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.CameraClass;

@TeleOp(name = "TestTeleop", group = "Test")
public class Test extends OpMode {
    public CameraClass cameraClass;
    public AdafruitI2cColorSensor adafruitI2cColorSensor;
    public NormalizedRGBA normalizedRGBA;
    public ElapsedTime runTime;


    @Override
    public void init() {

    }

    @Override
    public void loop() {
//        normalizedRGBA = adafruitI2cColorSensor.getNormalizedColors();

//        if (cameraClass.onceSeen)
//        {
//            if(cameraClass.tagState == CameraClass.TagState.Detected){
//                delta = cameraClass.cameraBearing;
////                fixed = m1.getCurrentPosition() - cameraClass.cameraBearing * outPutRes;
//
//                if (Math.abs(cameraClass.cameraBearing) > Math.toRadians(2)){
//                    vyrVoltage = turretPID.calculate(delta);
//                }
//            }else {
////                delta = (fixed - m1.getCurrentPosition()) / outPutRes;
//                delta = (Math.abs(cameraClass.cameraBearing) * outPutRes - m1.getCurrentPosition()) / outPutRes;
//                vyrVoltage = turretPID.calculate(delta);
//            }
//        }
//
//        if (normalizedRGBA.red > normalizedRGBA.green && normalizedRGBA.red > normalizedRGBA.blue && normalizedRGBA.red > 0.08){
//            vol = -0.1;
//        }
//




//        telemetry.addData("State", turretState.toString());
//        telemetry.addData("target", target);
//        telemetry.addData("Pow", m1.getPower());
//        telemetry.addData("ticks", m1.getCurrentPosition());
//        telemetry.addData("Rad/Degrees", Math.toDegrees(m1.getCurrentPosition() / outPutRes));
//        telemetry.addData("bool", goVyr);
//        telemetry.addData("vol", vol);
//        telemetry.addData("vyr", vyrVoltage);
//        telemetry.addData("red", normalizedRGBA.red);
//        telemetry.addData("green", normalizedRGBA.green);
//        telemetry.addData("blue", normalizedRGBA.blue);

//        telemetry.addData("updt time sec/Hz", "%.2f %.2f",runTime.seconds(),  1/ runTime.seconds());
        runTime.reset();
    }

}
