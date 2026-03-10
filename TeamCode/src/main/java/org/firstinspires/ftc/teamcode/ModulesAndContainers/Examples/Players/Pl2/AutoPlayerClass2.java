package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.Pl2;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Joysticks.Extenders.Joystick1;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
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
    public AutoPlayerClass2(MainFile mainFile, RobotClass robotClass) {
        super(mainFile);
        setJoystickActivityClass(new Joystick1(mainFile));

        this.hoodedShoter = robotClass.hoodedShoter;
        this.odometry = robotClass.odometry;

        pusher0 = new ServoMotorWrapper(mainFile, "pusher0");
        pusher1 = new ServoMotorWrapper(mainFile, "pusher1");
        pusher2 = new ServoMotorWrapper(mainFile, "pusher2");
        pusher3 = new ServoMotorWrapper(mainFile, "pusher3");
        pusher4 = new ServoMotorWrapper(mainFile, "pusher4");

        trackEmulator = new TrackEmulator(mainFile);
        speedController = new SpeedController(mainFile);
        pidfTunner = new PIDFTunner(mainFile);
    }
    public ServoMotorWrapper pusher0;
    public ServoMotorWrapper pusher1;
    public ServoMotorWrapper pusher2;
    public ServoMotorWrapper pusher3;
    public ServoMotorWrapper pusher4;
    public HoodedShoter hoodedShoter;
    public Odometry odometry;

    public TrackEmulator trackEmulator;
    public PIDFTunner pidfTunner;
    public SpeedController speedController;
    public ServoState servoState = ServoState.waiting;
    public enum ServoState{
        waiting,
        fired,
        prepared
    }
    @Override
    public void executeExt() {
        double collectorPow = 0;
        double turretPow = 0;
        double flyWheelPow = 0;

        double maxVol = 0.8;

        joystickActivityClass.update();

        pidfTunner.execute();

        trackEmulator.setPID(pidfTunner.getkP(), pidfTunner.getkI(), pidfTunner.getkD(), pidfTunner.getkF());

        double cosB = joystickActivityClass.cosB;
        double sinA = joystickActivityClass.sinA;

        double[] point = generalInformation.generalObjects.getPointVyr();
        //Выравниваем на ворота альянса

        OdometryData targetData = new OdometryData(new Position2D(point[0], point[1], point[3]), new Vector2(0), Math.abs(odometry.odometryBufferForRobot.read().getHeadVel()) + Math.toRadians(300));
        OdometryData currentData = new OdometryData(odometry.odometryBufferForTuret.read());

        cosB = trackEmulator.calculateVol(targetData, currentData);
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

                        hoodedShoter.angleController.execute(calclPos);

                        boolean isAngleGrowUp = hoodedShoter.angleController.getServo().isBusy();

                        //TODO условие на наводку турели
                        switch (servoState){
                            case waiting:
                                int index = 3 - hoodedShoter.digitalCellsClass.getArtifactCount();
                                int neededColor = odometry.cameraClass.motif[index];
                                hoodedShoter.digitalCellsClass.fire(neededColor);
                                if (hoodedShoter.digitalCellsClass.isStopped) {
                                    servoState = ServoState.fired;
                                }

                                break;
                            case fired:
                                if(!hoodedShoter.digitalCellsClass.triggeredServo.isBusy()) {
                                    hoodedShoter.digitalCellsClass.prepareServo();
                                    servoState = ServoState.prepared;
                                }
                                break;
                            case prepared:
                                if(!hoodedShoter.digitalCellsClass.triggeredServo.isBusy()) {
                                    hoodedShoter.digitalCellsClass.isStopped = false;
                                    servoState = ServoState.waiting;
                                }
                                break;
                        }

                        programState = ProgramState.Executing;
                    }
                    break;
                default:
                    break;
            }
        }else programState = ProgramState.Executing;

        checkButtons();

        cosB = Range.clip(cosB, -maxVol, maxVol);

        hoodedShoter.turretMotor.execute(cosB);
        hoodedShoter.flyWheelClass.execute(flyWheelPow);
        hoodedShoter.collector.execute(collectorPow);

        hoodedShoter.digitalCellsClass.setCurIterations(iterationCount);
        hoodedShoter.digitalCellsClass.update();
    }

    @Override
    protected void showDataExt() {
        telemetry.addData("motif", "%s %s %s",odometry.cameraClass.motif[0], odometry.cameraClass.motif[1], odometry.cameraClass.motif[2]);
        telemetry.addData("inTerr", hoodedShoter.digitalCellsClass.isStopped);
        joystickActivityClass.showData();
        hoodedShoter.showData();
        trackEmulator.showData();
        pidfTunner.showData();
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
    public class PIDFTunner extends PIDF {
        private double[] stepSize = {1, 0.1, 0.01, 0.001, 0.0001, 0.00001};
        private int stepIndex;
        private int index;

        public PIDFTunner(MainFile mainFile) {
            super(0.0, 0,0,0.08, -1,1, mainFile);
        }

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
                        kP += stepSize[stepIndex];
                        break;
                    case 1:
                        kI += stepSize[stepIndex];
                        break;
                    case 2:
                        kD += stepSize[stepIndex];
                        break;
                    case 3:
                        kF += stepSize[stepIndex];
                        break;
                }
                joystickActivityClass.dpad_Up = false;
            }

            if(joystickActivityClass.dpad_Down){
                switch (index){
                    case 0:
                        kP = Math.max(kP - stepSize[stepIndex], 0);
                        break;
                    case 1:
                        kI = Math.max(kI - stepSize[stepIndex], 0);
                        break;
                    case 2:
                        kD = Math.max(kD - stepSize[stepIndex], 0);
                        break;
                    case 3:
                        kF = Math.max(kF - stepSize[stepIndex], 0);
                        break;
                }
                joystickActivityClass.dpad_Down = false;
            }
        }

        @Override
        public void sayModuleName() {

        }

        @Override
        protected void showDataExt() {
            telemetry.addLine(String.format("globalIndex: %s index: %s stepSize: %s", pidfTunner.index, pidfTunner.stepIndex, pidfTunner.stepSize[pidfTunner.stepIndex]));
        }
    }

    public class TrackEmulator extends PIDF{
        public TrackEmulator(MainFile mainFile){
            super(0.3, 0.0,0,0, -1, 1, mainFile);
        }

        private double returnDistance(double VelMax, double accel ){
            return Math.pow(VelMax, 2) / (2 * accel);
        }
        public double targHeadVel;
        public double errorHeading;
        public double calculateVol(OdometryData targetData, OdometryData currentData){
            Position2D targetPos = targetData.getPosition();
            Position2D currentPos = currentData.getPosition();

            // Находим ошибку положения
            Position2D deltaPos = targetPos.minus(currentPos);

            double targHead = Math.atan2(
                    deltaPos.getY(),
                    deltaPos.getX());

            if(hoodedShoter.turretMotor.isNeedBack){
                double local = hoodedShoter.turretMotor.turretOdometry.localHead;
                errorHeading = -local;
//                errorHeading = -(local + Math.signum(local) * (Math.PI - Math.abs(odometry.odometryBufferForRobot.read().getPosition().getHeading())));
            }else {
                errorHeading = new Position2D(0,0, targHead - currentData.getPosition().getHeading()).getHeading();
            }
            double distanceBreak = returnDistance(targetData.getHeadVel(), Math.toRadians(1300));

            targHeadVel = Math.signum(errorHeading) * (Math.abs(errorHeading) > distanceBreak ? targetData.getHeadVel() : Math.toRadians(120)) ;

            double pidHeadVel = calculate(targHeadVel, currentData.getHeadVel());

            if(Math.abs(errorHeading) < Math.toRadians(2)) {
                pidHeadVel = 0;
                if(hoodedShoter.turretMotor.isNeedBack) hoodedShoter.turretMotor.isNeedBack = false;
            }

            return pidHeadVel;
        }
        @Override
        public void sayModuleName() {

        }
        @Override
        public void showData(){
            telemetry.addData("targVel", targHeadVel * RAD);
            telemetry.addData("error", "%.2f", errorHeading * RAD);
            showDataExt();
        }
    }
    public class SpeedController{
        public PIDF pidfFlyWheel;
        public SpeedController(MainFile mainFile){
            pidfFlyWheel = new PIDF(1, 1, 1,1,-1, 1, mainFile);
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

