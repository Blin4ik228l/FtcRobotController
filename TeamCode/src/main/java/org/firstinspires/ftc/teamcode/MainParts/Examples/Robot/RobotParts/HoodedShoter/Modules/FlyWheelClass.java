package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.Units;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.OdometryBuffer;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.ExecutableCollector;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableCollector;

public class FlyWheelClass extends ExecutableCollector {
    public ClassMath classMath;
    public OdometryBuffer flyWheelBuffer;
    public FlyWheelClass() {
        super(false);
        createMotorWrapperUtils();
        motorsCollector
                .add(motorBuilder.initialize(motorRight).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields(13.0, 1.0, 28.0, 4.0, 1.0).get())
                .add(motorBuilder.initialize(motorLeft).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields(13.0, 1.0, 28.0, 4.0, 1.0).get());

        classMath = new ClassMath(motorsCollector);
        sayCreated();
    }
    private String motorRight = controlHubDevices.getMotor(1);
    private String motorLeft = controlHubDevices.getMotor(2);

    public double getTargetSpeed(double theta, double range){
        double alpha = Math.toRadians(theta);
        double underRoot = 981 / (2 * (range * Math.tan(alpha) - 80));

        //Нужная скорость мяча в м/с
        double targetSpeed = (range / Math.cos(alpha)) * Math.sqrt(Math.abs(underRoot));

        //Если по формуле скоость отриц значит не стреляем
        if (underRoot < 0) return  0;

        targetSpeed = targetSpeed / CONTACT_PATCH;

        return targetSpeed;
    }
    @Override
    protected void executeExt(Double... args) {
        double power = args[0];

        motorsCollector.get(motorRight).execute(power);
        motorsCollector.get(motorLeft).execute(-power);
    }

    @Override
    protected void showDataExt() {
        motorsCollector.showData();
        classMath.showData();
    }
    public class ClassMath extends UpdatableCollector {
        private final OdometryData rawData;
        private double[] currentSpeeds;
        private double fltrdHeadVel, headAccel;
        private double curVel, deltaVel, lastVel;
        private double curTime, deltaTime, lastTime;
        private final ElapsedTime runTime;
        public ClassMath(MotorWrapper.InnerCollector motors) {
            super(false);
            motorsCollector = motors;
            fltrdHeadVel = 0.0;

            rawData = new OdometryData();
            flyWheelBuffer = new OdometryBuffer();
            currentSpeeds = new double[2];

            runTime = new ElapsedTime();
            sayCreated();
        }

        @Override
        protected void updateExt() {
            double filtr = 0.3;

            currentSpeeds[0] = -motorsCollector.get(motorLeft).getCurVel(Units.Rad);
            currentSpeeds[1] = motorsCollector.get(motorRight).getCurVel(Units.Rad);

            double vel = (currentSpeeds[0] != 0 && currentSpeeds[1] != 0 ? (currentSpeeds[0] - currentSpeeds[1]) / 2.0 : currentSpeeds[0] + currentSpeeds[1]);

            fltrdHeadVel = filtr * vel + (1 - filtr) * fltrdHeadVel;

            curVel = fltrdHeadVel;
            deltaVel = curVel - lastVel;
            lastVel = curVel;

            curTime = runTime.milliseconds();
            deltaTime = curTime - lastTime;
            lastTime = curTime;

            if(deltaTime == 0) headAccel = 0;
            else headAccel = deltaVel / deltaTime;

            rawData.setHeadVel(fltrdHeadVel);
            rawData.setHeadAccel(headAccel);

            flyWheelBuffer.beginWrite().set(rawData);
            flyWheelBuffer.endWrite2();
        }

        @Override
        protected void showDataExt() {
            telemetry.addData("speed L", currentSpeeds[0] * RAD);
            telemetry.addData("speed R", currentSpeeds[0]* RAD);
            telemetry.addData("speed", fltrdHeadVel * RAD);
        }
    }
}
