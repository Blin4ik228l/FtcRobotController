package org.firstinspires.ftc.teamcode.Programms.TeleOps.Other;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.CameraClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.NavigationSystem;

@TeleOp(name = "TestTeleop", group = "Test")
public class Test extends OpMode {
    public CameraClass cameraClass;
    public AdafruitI2cColorSensor adafruitI2cColorSensor;
    public NormalizedRGBA normalizedRGBA;
    public ElapsedTime runTime;
    public NavigationSystem navigationSystem;

    @Override
    public void init() {
//        cameraClass = new CameraClass(this);

        new GeneralInformation(GeneralInformation.ProgramName.TeleOp, GeneralInformation.Color.Red, GeneralInformation.StartPos.Nevermind);

        navigationSystem = new NavigationSystem(this);

//        adafruitI2cColorSensor = hardwareMap.get(AdafruitI2cColorSensor.class, "color");
//        try {
//            AMSColorSensor.class.getDeclaredField("AMS_TCS34725_ADDRESS").setAccessible(true);
//
//            AMSColorSensor.Parameters parameters = new AMSColorSensor.Parameters(AMS_TCS34725_ADDRESS, 0x4D);
//
//            Field paramField = I2cDeviceSynchDeviceWithParameters.class.getDeclaredField("parameters");
//
//            paramField.setAccessible(true);
//
//            try {
//                paramField.set(adafruitI2cColorSensor, parameters);
//            } catch (Exception e) {
//                throw new RuntimeException(e);
//            }
//
//            adafruitI2cColorSensor.initialize();
//        } catch (NoSuchFieldException e) {
//            throw new RuntimeException("color sensor hack not successful");
//        }

        runTime = new ElapsedTime();

//        adafruitI2cColorSensor.setGain(15f);

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
        navigationSystem.showData();
//        telemetry.addData("updt time sec/Hz", "%.2f %.2f",runTime.seconds(),  1/ runTime.seconds());
        runTime.reset();
    }

    @Override
    public void stop() {
        navigationSystem.odometry.thread.interrupt();
    }
}
