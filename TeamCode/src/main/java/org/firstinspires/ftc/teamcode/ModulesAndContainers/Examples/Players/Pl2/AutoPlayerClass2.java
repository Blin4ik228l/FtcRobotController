package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Joysticks.Extenders.Joystick1;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.PIDF;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.HoodedShoter;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.ProgramState;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ServoMotorWrapper;

public class AutoPlayerClass2 extends PlayerClass{
    public AutoPlayerClass2(GeneralInformation generalInformation, RobotClass robotClass, OpMode op) {
        super(generalInformation,"Player2", op);
        setJoystickActivityClass(new Joystick1(op));
        this.hoodedShoter = robotClass.hoodedShoter;
        this.odometry = robotClass.odometry;

        pusher0 = new ServoMotorWrapper(op, "pusher0");
        pusher1 = new ServoMotorWrapper(op, "pusher1");
        pusher2 = new ServoMotorWrapper(op, "pusher2");
        pusher3 = new ServoMotorWrapper(op, "pusher3");
        pusher4 = new ServoMotorWrapper(op, "pusher4");

        trackEmulator = new TrackEmulator(op);
        speedController = new SpeedController(op);
        pidfTunner = new PIDFTunner();
        updateTime = new ElapsedTime();
    }
    public ServoMotorWrapper pusher0;
    public ServoMotorWrapper pusher1;
    public ServoMotorWrapper pusher2;
    public ServoMotorWrapper pusher3;
    public ServoMotorWrapper pusher4;
    public HoodedShoter hoodedShoter;
    public Odometry odometry;
    public ElapsedTime updateTime;
    public double hz;
    public TrackEmulator trackEmulator;
    public PIDFTunner pidfTunner;
    public SpeedController speedController;

    @Override
    public ProgramState execute(){
        double collectorPow = 0;
        double turretPow = 0;
        double flyWheelPow = 0;

        joystickActivityClass.update();

        pidfTunner.execute();

        trackEmulator.trackingPIDF.setPID(pidfTunner.P, pidfTunner.I, pidfTunner.D, pidfTunner.F);

        double cosB = joystickActivityClass.cosB;

        double targSpeed = 50 * (joystickActivityClass.tAPressed % 4);

        if(hoodedShoter.turretMotor.isInterrupted){
           targSpeed = Math.toRadians(1000);
        }else targSpeed = odometry.odometryBufferForRobot.read().getHeadVel() + Math.toRadians(50);

        //Выравниваем на ворота альянса
        double[] point = generalInformation.generalObjects.getPointVyr();
        OdometryData targetData = new OdometryData(new Position2D(point[0], point[1], point[3]), new Vector2(0), targSpeed);
        OdometryData currentData = new OdometryData(odometry.odometryBufferForTuret.read());

        turretPow = trackEmulator.calculateVol(targetData, currentData);
        double curSpeed;
        double targetSpeed;
        if(!isInterrupted){
            switch (generalInformation.gameTactick){
                case Load:
                    curSpeed = hoodedShoter.flyWheelClass.flyWheelOdometry.odometryData.getHeadVel();
                    targetSpeed = 0;
                    flyWheelPow = speedController.calculateVol(targetSpeed, curSpeed);

                    if (hoodedShoter.digitalCellsClass.getArtifactCount() == 3){
                        collectorPow = -1;
                        programState = ProgramState.Finished;
                    }else {
                        collectorPow = 1;
                        programState = ProgramState.Executing;
                    }
                    break;
                case Fire:
                    collectorPow = 0;
                    if (hoodedShoter.digitalCellsClass.getArtifactCount() == 0){
                        hoodedShoter.digitalCellsClass.prepareServo();//Возвращаем серво в начальное положение
                        programState = ProgramState.Finished;
                    }else {
                        double range = new Vector2(point[0] - odometry.odometryBufferForTuret.read().getPosition().getX(), point[1] - odometry.odometryBufferForTuret.read().getPosition().getY()).length();

                        double theta;

                        //в градусах
                        if (range >= 290) theta = 50;
                        else if (range >= 150) theta = 60;
                        else if (range >= 70) theta = 70;
                        else theta = 80;

                        targetSpeed = hoodedShoter.flyWheelClass.getTargetSpeed(theta, range);
                        curSpeed = hoodedShoter.flyWheelClass.flyWheelOdometry.odometryData.getHeadVel();

                        flyWheelPow = speedController.calculateVol(targetSpeed, curSpeed);

                        boolean isFlyWheelReady = speedController.checkReadnees(targetSpeed, curSpeed);

                        double calclPos = hoodedShoter.angleController.getPos(theta);
                        boolean isAngleGrowUp = hoodedShoter.angleController.servoMotorWrapper.setSignal(calclPos);

                        //TODO условие на наводку турели
                        if(isFlyWheelReady && isAngleGrowUp){
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
                default:
                    break;
            }
        }else programState = ProgramState.Executing;

        checkButtons();

        hoodedShoter.turretMotor.setPower(turretPow);
        hoodedShoter.flyWheelClass.setPower(flyWheelPow);
        hoodedShoter.collector.setPower(collectorPow);

        hoodedShoter.update();

        hz = 1 / updateTime.seconds();
        updateTime.reset();
        return programState;
    }
    @Override
    public void showData(){
        telemetry.addLine("===Player2===");
        telemetry.addData("Update time/hz", hz);
        joystickActivityClass.showData();
        hoodedShoter.showData();
        trackEmulator.trackingPIDF.showData();
        telemetry.addLine(String.format("globalIndex: %s index: %s stepSize: %s", pidfTunner.index, pidfTunner.stepIndex, pidfTunner.stepSize[pidfTunner.stepIndex]));
        telemetry.addLine(String.format("targHead %.2f",trackEmulator.targHead * RAD));
        telemetry.addLine(String.format("curHead %.2f", odometry.odometryBufferForTuret.read().getPosition().getHeading() * RAD));
        telemetry.addLine();
    }
    @Override
    public void buttonAReleased() {
        pusher0.setPosition(0);

    }

    @Override
    public void buttonAUnReleased() {
        pusher0.setPosition(0.37);
    }

    @Override
    public void buttonBReleased() {
        pusher1.setPosition(0.37);
    }

    @Override
    public void buttonBUnReleased() {
        pusher1.setPosition(0);
    }

    @Override
    public void buttonXReleased() {
        pusher2.setPosition(0.37);
    }

    @Override
    public void buttonXUnReleased() {
        pusher2.setPosition(0);
    }

    @Override
    public void buttonYReleased() {

    }

    @Override
    public void buttonYUnReleased() {

    }
    public class PIDFTunner{
        private double  P = 0.002, I = 0, D = 0, F = 0.05;
        private double[] stepSize = {1, 0.1, 0.01, 0.001, 0.0001, 0.00001};
        private int stepIndex;
        private int index;
        public void execute(){
            if(joystickActivityClass.bumperLeft){
                stepIndex = Math.max(stepIndex - 1, 0);
                joystickActivityClass.bumperLeft = false;
            }

            if(joystickActivityClass.bumperRight){
                stepIndex = (stepIndex + 1) % stepSize.length;
                joystickActivityClass.bumperRight = false;
            }

            if(joystickActivityClass.triggerLeft){
                index = Math.max(index - 1, 0);
                joystickActivityClass.triggerLeft = false;
            }

            if(joystickActivityClass.triggerRight){
                index = (index + 1) % 4;
                joystickActivityClass.triggerRight = false;
            }

            if(joystickActivityClass.dpad_Up){
                switch (index){
                    case 0:
                        P += stepSize[stepIndex];
                        break;
                    case 1:
                        I += stepSize[stepIndex];
                        break;
                    case 2:
                        D += stepSize[stepIndex];
                        break;
                    case 3:
                        F += stepSize[stepIndex];
                        break;
                }
                joystickActivityClass.dpad_Up = false;
            }

            if(joystickActivityClass.dpad_Down){
                switch (index){
                    case 0:
                        P = Math.max(P - stepSize[stepIndex], 0);
                        break;
                    case 1:
                        I = Math.max(I - stepSize[stepIndex], 0);
                        break;
                    case 2:
                        D = Math.max(D - stepSize[stepIndex], 0);
                        break;
                    case 3:
                        F = Math.max(F - stepSize[stepIndex], 0);
                        break;
                }
                joystickActivityClass.dpad_Down = false;
            }
        }

        public double getP() {
            return P;
        }

        public double getI() {
            return I;
        }

        public double getD() {
            return D;
        }

        public double getF() {
            return F;
        }
    }

    public class TrackEmulator {
        private PIDF trackingPIDF;
        public TrackEmulator(OpMode op){
            this.trackingPIDF = new PIDF(0.09, 0.000001,0,0, -1, 1, op);
        }
        private double returnDistance(double VelMax, double accel ){
            return Math.pow(VelMax, 2) / (2 * accel);
        }
        public double targHead, targHeadVel;
        public double calculateVol(OdometryData targetData, OdometryData currentData){
            Position2D targetPos = targetData.getPosition();
            Position2D currentPos = currentData.getPosition();

            // Находим ошибку положения
            Position2D deltaPos = targetPos.minus(currentPos);

            double errorHeading;
            double headVel;
            double distanceBreak;

            //Смотрим прямо на объект
            targHead = Math.atan2(
                    deltaPos.getY(),
                    deltaPos.getX()
            );

            //Если турель сделал 1 полный оборот то резко крутимя обратно
            if(hoodedShoter.turretMotor.isInterrupted){
                errorHeading = 0 - (hoodedShoter.turretMotor.turretOdometry.localHead - targHead);

//                headVel = targetData.getHeadVel() * Math.signum(errorHeading);
                distanceBreak = returnDistance(targetData.getHeadVel(), Math.toRadians(3000));
            }else {
                errorHeading = new Position2D(0, 0, targHead - currentData.getPosition().getHeading()).getHeading();

                distanceBreak = returnDistance(targetData.getHeadVel(), Math.toRadians(300));
            }

            targHeadVel = Math.signum(errorHeading) * (Math.abs(errorHeading) > distanceBreak ? targetData.getHeadVel() : 0.05) ;
            double pidHeadVel = trackingPIDF.calculate(targHeadVel, currentData.getHeadVel());

            if(Math.abs(errorHeading) < Math.toRadians(1.5)) {
                pidHeadVel = 0;
                if(hoodedShoter.turretMotor.isInterrupted) hoodedShoter.turretMotor.isInterrupted = false;
            }

            return pidHeadVel;
        }
    }
    public class SpeedController{
        public PIDF pidfFlyWheel;
        public SpeedController(OpMode op){
            pidfFlyWheel = new PIDF(1, 1, 1,1,-1, 1, op);
        }
        public double calculateVol(double targetSpeed, double curSpeed){
            double pidPower = pidfFlyWheel.calculate(targetSpeed, curSpeed);

            return pidPower;
        }
        public boolean checkReadnees(double targetSpeed, double curSped){
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

