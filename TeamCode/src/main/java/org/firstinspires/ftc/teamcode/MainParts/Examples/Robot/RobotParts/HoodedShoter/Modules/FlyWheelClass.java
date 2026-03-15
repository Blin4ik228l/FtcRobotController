package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.Units;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.InnerMath;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.ExecutableCollector;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableCollector;

public class FlyWheelClass extends ExecutableCollector {
    public FlyWheelOdometry flyWheelOdometry;
    public FlyWheelClass() {
        createMotorWrapperUtils();
        motorsCollector
                .add(motorBuilder.initialize(motorRight).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields(13.0, 1.0).get())
                .add(motorBuilder.initialize(motorLeft).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields(13.0, 1.0).get());

        flyWheelOdometry = new FlyWheelOdometry(motorsCollector);
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
        flyWheelOdometry.showData();
    }
    public class FlyWheelOdometry extends UpdatableCollector {
        public OdometryData odometryData;
        public SelfMath selfMath;
        public FlyWheelOdometry(MotorWrapper.InnerCollector motors){
            motorsCollector = motors;
            odometryData = new OdometryData();
            selfMath = new SelfMath();
            sayCreated();
        }

        @Override
        protected void updateExt() {
            selfMath.updateAll();
        }

        @Override
        protected void showDataExt() {
            telemetry.addData("speed L", selfMath.currentSpeeds[0] * RAD);
            telemetry.addData("speed R", selfMath.currentSpeeds[0]* RAD);
            telemetry.addData("speed", selfMath.filteredVel * RAD);
        }

        public class SelfMath extends InnerMath {
            private double[] currentSpeeds;
            public double curentSpeedAll;
            public double filteredVel;

            public SelfMath(){
                setRadius(4).setDRIVE_GEAR_REDUCTION(1).setCOUNTS_PER_ENCODER_REV(28).calculateCountsPerCm();
                currentSpeeds = new double[2];
            }

            public void updateAll(){
                updateSpeed();
            }
            public void updateSpeed(){
                double filtr = 0.3;

                currentSpeeds[0] = getCurrentVelocity(motorsCollector.get(motorLeft), Units.Rad);
                currentSpeeds[1] = getCurrentVelocity(motorsCollector.get(motorRight), Units.Rad);

                double vel = -(currentSpeeds[0] != 0 && currentSpeeds[1] != 0 ? (currentSpeeds[0] + currentSpeeds[1]) / 2.0 : currentSpeeds[0] + currentSpeeds[1]);

                filteredVel = filtr * vel + (1 - filtr) * filteredVel;

                odometryData.setHeadVel(filteredVel);
            }
        }

    }
}
