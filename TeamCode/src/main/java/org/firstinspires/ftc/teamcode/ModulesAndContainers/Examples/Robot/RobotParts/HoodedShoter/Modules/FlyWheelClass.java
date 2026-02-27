package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.Units;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.MotorModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;

public class FlyWheelClass extends MotorModule {
    public FlyWheelOdometry flyWheelOdometry;
    public FlyWheelClass(OpMode op, VoltageSensorClass voltageSensorClass) {
        super(op);
        motorsWrapper
                .add(motorBuilder.initialize(op, motorRight).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields(voltageSensorClass, 12.5, 1).get())
                .add(motorBuilder.initialize(op, motorLeft).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields(voltageSensorClass, 12.5, 1).get());

//        motorsWrapper.get(motorLeft).getMotorConfigurationType().setMaxRPM(1111);

        flyWheelOdometry = new FlyWheelOdometry(op);
        sayInited();
    }
    private String motorRight = controlHubDevices.getMotor(1);
    private String motorLeft = controlHubDevices.getMotor(2);

    public double getTargetSpeed(double theta, double range){
        double alpha = Math.toRadians(theta);
        double underRoot = 981 / (2 * (range * Math.tan(alpha) - 80));


        double targetSpeed = (range / Math.cos(alpha)) * Math.sqrt(Math.abs(underRoot)) / 100;

        //Если по формуле скоость отриц значит не стреляем
        if (underRoot < 0) return  0;

        //Перевод в радианы/сек
        targetSpeed = (targetSpeed / MAX_EXPERIMENTAL_SPEED_IN_METERS * MAX_RAD_SPEED) * 19.2;

        return targetSpeed;
    }
    public void setPower(double power){
        motorsWrapper.get(motorRight).setPower(power);
        motorsWrapper.get(motorLeft).setPower(-power);

        flyWheelOdometry.update();
    }

    @Override
    public void showData() {
        motorsWrapper.showData();
    }
    public class FlyWheelOdometry extends UpdatableModule {
        public OdometryData odometryData;
        public SelfMath selfMath;
        public boolean switcher = false;
        public FlyWheelOdometry(OpMode op){
            super(op);
            odometryData = new OdometryData();
            selfMath = new SelfMath();
        }

        @Override
        public void update() {
            selfMath.updateAll();
        }
        @Override
        public void showData() {

        }

        public class SelfMath{
            private double[] currentSpeeds;
            public double curentSpeedAll;
            public double filteredVel;

            public SelfMath(){
                currentSpeeds = new double[2];
            }

            public void updateAll(){
                updateSpeed();
            }
            public void updateSpeed(){
                double filtr = 0.3;

                currentSpeeds[0] = innerMath.getCurrentVelocity(motorLeft, Units.Rad);
                currentSpeeds[1] = innerMath.getCurrentVelocity(motorRight, Units.Rad);

                double vel = currentSpeeds[0] != 0 && currentSpeeds[1] != 0 ? (currentSpeeds[0] + currentSpeeds[1]) / 2.0 : currentSpeeds[0] + currentSpeeds[1];

                filteredVel = filtr * vel + (1 - filtr) * filteredVel;

                odometryData.setHeadVel(filteredVel);
            }
        }

    }
}
