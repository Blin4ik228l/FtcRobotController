package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.MotorModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;

public class FlyWheelClass extends MotorModule {
    public FlyWheelOdometry flyWheelOdometry;
    public FlyWheelClass(OpMode op, VoltageSensorClass voltageSensorClass) {
        super(op);
        motorsWrapper
                .add(op, motorBuilder.initialize(op, motorRight).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields(voltageSensorClass, 12.5, 1).get())
                .add(op, motorBuilder.initialize(op, motorLeft).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields(voltageSensorClass, 12.5, 1).get());

        motorsWrapper.get(motorLeft).getMotorConfigurationType().setMaxRPM(1111);

        flyWheelOdometry = new FlyWheelOdometry(op);
        sayInited();
    }
    private String motorRight = expansionHubDevices.getMotor(0);
    private String motorLeft = expansionHubDevices.getMotor(1);

    public double getTargetSpeed(double theta, double range){
         double alpha = Math.toRadians(theta);

        double targetSpeed = (range / Math.cos(alpha)) * Math.sqrt(Math.abs(981 / (2 * (range * Math.tan(alpha) - 80)))) / 100;

        //Если по формуле скоость отриц значит не стреляем
        if (981 / (2 * (range * Math.tan(alpha) - 80)) < 0) targetSpeed = 0;

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
        public double flyWheelRadius = 0.4;
        private double COUNTS_PER_ENCODER_REV = motorsWrapper.get(motorLeft).getMotorConfigurationType().getTicksPerRev();
        private double DRIVE_GEAR_REDUCTION = 1;
        private double COUNTS_PER_CM = (COUNTS_PER_ENCODER_REV * DRIVE_GEAR_REDUCTION)/
                (flyWheelRadius * 2 * Math.PI);
        private double ticksToCm(double ticks){
            return ticks / COUNTS_PER_CM;
        }
        public double getCurentPos(String motorName){
            DcMotor motor = motorsWrapper.get(motorName).getMotor();

            if(motor == null) return 0;
            else {
                double pos = -motor.getCurrentPosition();
                return switcher ? ticksToCm(pos) : ticksToCm(pos) / flyWheelRadius;
            }
        }
        public double getCurrentVelocity(String motorName){
            DcMotorEx motorEx = motorsWrapper.get(motorName).getMotorEx();

            if(motorEx == null) return 0;
            else {
                double vel = -motorEx.getVelocity();
                return switcher ? ticksToCm(vel) : ticksToCm(vel) / flyWheelRadius;
            }
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

                currentSpeeds[0] = getCurrentVelocity(motorLeft);
                currentSpeeds[1] = getCurrentVelocity(motorRight);

                double vel = currentSpeeds[0] != 0 && currentSpeeds[1] != 0 ? (currentSpeeds[0] + currentSpeeds[1]) / 2.0 : currentSpeeds[0] + currentSpeeds[1];

                filteredVel = filtr * vel + (1 - filtr) * filteredVel;

                odometryData.setHeadVel(filteredVel);
            }
        }

    }
}
