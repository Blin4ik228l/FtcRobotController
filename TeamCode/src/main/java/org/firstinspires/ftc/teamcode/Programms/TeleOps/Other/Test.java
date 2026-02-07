package org.firstinspires.ftc.teamcode.Programms.TeleOps.Other;

import static com.qualcomm.hardware.ams.AMSColorSensor.AMS_TCS34725_ADDRESS;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRColor;
import org.firstinspires.ftc.teamcode.Robot.GeneralInformation;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.CameraClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.PID;
import org.opencv.core.Mat;

import java.lang.reflect.Field;
import java.security.PublicKey;
@TeleOp(name = "TestTeleop", group = "Test")
public class Test extends OpMode {
    public CameraClass cameraClass;
    public DcMotor m1;
    public AdafruitI2cColorSensor adafruitI2cColorSensor;
    public NormalizedRGBA normalizedRGBA;
    public PID turretPID;
    public ElapsedTime runTime;
    public double targ;
    public boolean goVyr = false;
    public boolean goVyr2 = false;
    public double fixed = 0;
    public double lastMotorPos, curMotorPos, deltaPos;
    public double deltaAngle;
    public TurretState turretState = TurretState.Rotate_to_target;
    public enum TurretState{
        LOST,
        Return_to_zero,
        Rotate_to_target,
        Idle
    }
    @Override
    public void init() {
        cameraClass = new CameraClass(this);
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        adafruitI2cColorSensor = hardwareMap.get(AdafruitI2cColorSensor.class, "color");
        try {
            AMSColorSensor.class.getDeclaredField("AMS_TCS34725_ADDRESS").setAccessible(true);

            AMSColorSensor.Parameters parameters = new AMSColorSensor.Parameters(AMS_TCS34725_ADDRESS, 0x4D);

            Field paramField = I2cDeviceSynchDeviceWithParameters.class.getDeclaredField("parameters");

            paramField.setAccessible(true);

            try {
                paramField.set(adafruitI2cColorSensor, parameters);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }

            adafruitI2cColorSensor.initialize();
        } catch (NoSuchFieldException e) {
            throw new RuntimeException("color sensor hack not successful");
        }

        runTime = new ElapsedTime();

        adafruitI2cColorSensor.setGain(15f);
        turretPID = new PID(0.01, 0.0001,0, -1, 1);
    }

    @Override
    public void loop() {
        cameraClass.update();
        normalizedRGBA = adafruitI2cColorSensor.getNormalizedColors();

        double outPutRes = 288;
        double delta = 0;
        double divider = Math.PI;
        double vyrVoltage = 0;
        double vol = 0;

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


        double target = 0;
        double p = 0;

        curMotorPos = m1.getCurrentPosition();
        deltaPos = lastMotorPos - curMotorPos;
        lastMotorPos = curMotorPos;

        switch (cameraClass.tagState){
            case Detected:
                target = cameraClass.cameraBearing;
                break;
            case UnDetected:
                deltaAngle += deltaPos / outPutRes;
                target = cameraClass.cameraBearing + deltaAngle;
                break;
        }
        switch (turretState){
            case Idle:
                deltaAngle = 0;
                turretState = TurretState.Rotate_to_target;
                break;
            case Return_to_zero:
                target = 0 - m1.getCurrentPosition() / outPutRes;
                break;
            case Rotate_to_target:

                vyrVoltage = turretPID.calculate(target);
                break;
            case LOST:
                break;
        }
        if (Math.abs(m1.getCurrentPosition()) > outPutRes){
            turretState = TurretState.Return_to_zero;
            return;
        }
        if (Math.abs(target) < Math.toRadians(3) && cameraClass.onceSeen) {
            vyrVoltage = 0;
            turretState = TurretState.Idle;
        }
        m1.setPower(vyrVoltage);

        cameraClass.showData();
        telemetry.addData("State", turretState.toString());
        telemetry.addData("target", target);
        telemetry.addData("Pow", m1.getPower());
        telemetry.addData("ticks", m1.getCurrentPosition());
        telemetry.addData("Rad/Degrees", Math.toDegrees(m1.getCurrentPosition() / outPutRes));
        telemetry.addData("bool", goVyr);
        telemetry.addData("vol", vol);
        telemetry.addData("vyr", vyrVoltage);
//        telemetry.addData("red", normalizedRGBA.red);
//        telemetry.addData("green", normalizedRGBA.green);
//        telemetry.addData("blue", normalizedRGBA.blue);
        telemetry.addData("updateTime", runTime.seconds());
        runTime.reset();
    }
}
