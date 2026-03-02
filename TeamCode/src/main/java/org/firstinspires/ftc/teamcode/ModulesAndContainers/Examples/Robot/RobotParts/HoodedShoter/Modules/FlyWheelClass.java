package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.MainSystem;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.Units;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.InnerMath;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.ExecutingModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatingModule;

public class FlyWheelClass extends ExecutingModule {
    public FlyWheelOdometry flyWheelOdometry;
    public FlyWheelClass(MainFile mainFile) {
        super(mainFile);
        createMotorWrapperUtils();
        motorsCollector
                .add(motorBuilder.initialize(mainFile, motorRight).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields(12.5, 1.0).get())
                .add(motorBuilder.initialize(mainFile, motorLeft).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields(12.5, 1.0).get());

//        motorsWrapper.get(motorLeft).getMotorConfigurationType().setMaxRPM(1111);

        flyWheelOdometry = new FlyWheelOdometry(mainFile);
        sayCreated();
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
    @Override
    protected void executeExt(Double... args) {
        double power = args[0];
        motorsCollector.get(motorRight).execute(power);
        motorsCollector.get(motorLeft).execute(-power);

        flyWheelOdometry.update();
    }

    @Override
    protected void showDataExt() {
        motorsCollector.showData();
    }
    public class FlyWheelOdometry extends UpdatingModule {
        public OdometryData odometryData;
        public SelfMath selfMath;
        public FlyWheelOdometry(MainFile mainFile){
            super(mainFile);
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

        }
        public class SelfMath extends InnerMath {
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

                currentSpeeds[0] = getCurrentVelocity(motorsCollector.get(motorLeft), Units.Rad);
                currentSpeeds[1] = getCurrentVelocity(motorsCollector.get(motorRight), Units.Rad);

                double vel = currentSpeeds[0] != 0 && currentSpeeds[1] != 0 ? (currentSpeeds[0] + currentSpeeds[1]) / 2.0 : currentSpeeds[0] + currentSpeeds[1];

                filteredVel = filtr * vel + (1 - filtr) * filteredVel;

                odometryData.setHeadVel(filteredVel);
            }
        }

    }
}
