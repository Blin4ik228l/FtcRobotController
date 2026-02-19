package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.AnotherConsts;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Joysticks.Extenders.Joystick1;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Joysticks.Extenders.Joystick2;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Odometry.Parts.MathUtils.PIDF;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.HoodedShoter;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.AngleController;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.Collector;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.DigitalCellsClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.FlyWheelClass;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.ProgramState;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.ServoMotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.ExecutableModule;

import java.util.ArrayList;

public class AutoPlayerClass2 extends PlayerClass{
    public AutoPlayerClass2(GeneralInformation generalInformation, RobotClass robotClass, OpMode op) {
        super(generalInformation, "Player2",op);
        this.hoodedShoter = robotClass.hoodedShoter;
        this.odometry = robotClass.odometry;

        setJoystickActivityClass(new Joystick1(op));
    }

    public HoodedShoter hoodedShoter;
    public Odometry odometry;
    public TrackEmulator trackEmulator;
    public SpeedController speedController;
    public boolean isInterrupted;

    @Override
    public ProgramState execute(){
        if(isInterrupted) return ProgramState.Interrupted;
        double collectorPow = 0;
        double turretPow = 0;
        double flyWheelPow = 0;

        double[] point = generalInformation.generalObjects.getPointVyr();
        //Выравниваем на ворота альянса
        OdometryData targetData = new OdometryData(new Position2D(point[0], point[1], 0), new Vector2(0), 30);
        OdometryData currentData = new OdometryData(odometry.odometryBuffer.read().getRobotPosition(), new Vector2(0), hoodedShoter.turretMotor.turretOdometry.headVel);

        turretPow = trackEmulator.calculateVol(targetData, currentData);

        switch (generalInformation.gameState){
            case Load:
                flyWheelPow = speedController.calculateVol(0, hoodedShoter.flyWheelClass.curentSpeed);

                if (hoodedShoter.digitalCellsClass.getArtifactCount() == 3){
                    collectorPow = -1;
                    programState = ProgramState.Finished;
                }else {
                    collectorPow = 1;
                    programState = ProgramState.Executing;
                }
                break;
            default:
                collectorPow = 0;
                if (hoodedShoter.digitalCellsClass.getArtifactCount() == 0){
                    hoodedShoter.digitalCellsClass.prepareServo();//Возвращаем серво в начальное положение
                    programState = ProgramState.Finished;
                }else {
                    double range = new Vector2(point[0] - odometry.odometryBuffer.read().getRobotPosition().getX(), point[1] - odometry.odometryBuffer.read().getRobotPosition().getY()).length();

                    double theta;

                    //в градусах
                    if (range >= 290) theta = 50;
                    else if (range >= 150) theta = 60;
                    else if (range >= 70) theta = 70;
                    else theta = 80;

                    double alpha = Math.toRadians(theta);

                    double targetSpeed = (range / Math.cos(alpha)) * Math.sqrt(Math.abs(981 / (2 * (range * Math.tan(alpha) - 80)))) / 100;

                    //Если по формуле скоость отриц значит не стреляем
                    if (981 / (2 * (range * Math.tan(alpha) - 80)) < 0) targetSpeed = 0;

                    targetSpeed = (targetSpeed / MAX_EXPERIMENTAL_SPEED_IN_METERS * MAX_RAD_SPEED) * 19.2;

                    flyWheelPow = speedController.calculateVol(targetSpeed, hoodedShoter.flyWheelClass.curentSpeed);
                    boolean isAngleGrowUp = hoodedShoter.angleController.setAngle(theta);

                    if(speedController.checkReadnees(hoodedShoter.flyWheelClass.curentSpeed, targetSpeed) && isAngleGrowUp){
                        if(hoodedShoter.digitalCellsClass.isInerrupted){
                            if(!hoodedShoter.digitalCellsClass.triggeredServo.isBusy()) hoodedShoter.digitalCellsClass.isInerrupted = false;
                        }

                        int index = 3 - hoodedShoter.digitalCellsClass.getArtifactCount();
                        int neededColor = odometry.cameraClass.motif[index];
                        hoodedShoter.digitalCellsClass.fire(neededColor);
                    }
                    programState = ProgramState.Executing;
                }
                break;
        }

        checkButtons();

        hoodedShoter.turretMotor.setPower(turretPow);
        hoodedShoter.flyWheelClass.setPower(flyWheelPow);
        hoodedShoter.collector.setPower(collectorPow);
        return programState;
    }
    @Override
    public void buttonAReleased() {

    }

    @Override
    public void buttonAUnReleased() {

    }

    @Override
    public void buttonBReleased() {

    }

    @Override
    public void buttonBUnReleased() {

    }

    @Override
    public void buttonXReleased() {

    }

    @Override
    public void buttonXUnReleased() {

    }

    @Override
    public void buttonYReleased() {

    }

    @Override
    public void buttonYUnReleased() {

    }

    @Override
    public void showData(){
        telemetry.addLine("===AUTO PLAYER===");

        telemetry.addLine();
    }
    public class TrackEmulator {
        private PIDF trackingPIDF;
        public TrackEmulator(){
            this.trackingPIDF = new PIDF(0.45, 0.0,0,0, -1, 1);
        }
        private double returnDistance(double VelMax, double accel ){
            return Math.pow(VelMax, 2) / (2 * accel);
        }
        public double calculateVol(OdometryData targetData, OdometryData currentData){

            Position2D targetPos = targetData.getRobotPosition();
            Position2D currentPos = currentData.getRobotPosition();

            // Находим ошибку положения
            Position2D deltaPos = targetPos.minus(currentPos);

            double errorHeading;

            //Если турель сделал 1 полный оборот то резко крутимя обратно
            if(hoodedShoter.turretMotor.isInterrupted){
                errorHeading = hoodedShoter.turretMotor.turretOdometry.turrelLocalHead;
            }else {
                errorHeading = Math.atan2(
                        deltaPos.getY(),
                        deltaPos.getX()
                );
            }
            //Смотрим прямо на объект


            double headVel = errorHeading > returnDistance(targetData.getRobotHeadVel() , MAX_ANGULAR_ACCEL) ? targetData.getRobotHeadVel() : MIN_ANGULAR_SPEED;

            double pidHeadVel = trackingPIDF.calculate(headVel);

            if(Math.abs(errorHeading) < Math.toRadians(1.5)) {
                pidHeadVel = 0;
                if(hoodedShoter.turretMotor.isInterrupted) hoodedShoter.turretMotor.isInterrupted = false;
            }

            return pidHeadVel;
        }
    }
    public class SpeedController{
        public PIDF pidfFlyWheel;
        public SpeedController(){
            pidfFlyWheel = new PIDF(1, 1, 1,1,-1, 1);
        }
        public double calculateVol(double targetSpeed, double curSpeed){

            double pidPower = pidfFlyWheel.calculate(targetSpeed, curSpeed);

            return pidPower;
        }
        public boolean checkReadnees(double curSped, double targetSpeed){
            double errorPart = Math.abs(curSped / targetSpeed - 1);

            if (errorPart > 0.01)
            {
                return false;
            }
            else {
                return true;
            }
        }
    }
}

