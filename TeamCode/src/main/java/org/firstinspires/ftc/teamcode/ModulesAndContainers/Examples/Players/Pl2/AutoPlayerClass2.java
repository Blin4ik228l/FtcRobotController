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
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

public class AutoPlayerClass2 extends PlayerClass{
    public AutoPlayerClass2(MainFile mainFile, RobotClass robotClass) {
        super(mainFile);
        setJoystickActivityClass(new Joystick1(mainFile));

        this.hoodedShoter = robotClass.hoodedShoter;
        this.odometry = robotClass.odometry;

        trackEmulator = new TrackEmulator(mainFile);
        speedController = new SpeedController(mainFile);
        pidfTunner = new PIDFTunner(mainFile);
    }
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

    boolean isFlyWheelReady;
    boolean isAngleGrowUp;
    private double headVoltage;
    @Override
    public void executeExt() {
        double collectorPow = 0;
        double turretPow = 0;
        double flyWheelPow = 0;

        double maxVol = 0.8;

        joystickActivityClass.update();

//        pidfTunner.execute();
//
//        speedController.pidfFlyWheel.setPID(pidfTunner.getkP(), pidfTunner.getkI(), pidfTunner.getkD(), pidfTunner.getkF());

        double cosB;
        double sinA;

        //Выравниваем на ворота альянса
        double[] point = generalInformation.generalObjects.getPointVyr();
        OdometryData targetData;
        OdometryData currentData = new OdometryData(odometry.odometryBufferForTuret.read());

        Position2D targPos = new Position2D(point[0], point[1], 0);
        Position2D curPos = currentData.getPosition();

        Position2D deltaPos = targPos.minus(curPos);
        double targHead = new Position2D(0,0, Math.atan2(
                deltaPos.getY(),
                deltaPos.getX())).getHeading();

        targetData = new OdometryData(new Position2D(0,0, targHead), new Vector2(0), MAX_TURRET_HEAD_SP);

        cosB = trackEmulator.calculateVol(targetData, currentData);

        if(!isInterrupted){
            double curSpeed;
            double targetSpeed;

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
                    }
                    hoodedShoter.digitalCellsClass.setCurIterations(iterationCount);
                    hoodedShoter.digitalCellsClass.update();
                    break;
                case Fire:
                    if (hoodedShoter.digitalCellsClass.getArtifactCount() == 0){
                        hoodedShoter.digitalCellsClass.prepareServo();//Возвращаем серво в начальное положение
                        programState = ProgramState.Finished;
                    }else {
                        collectorPow = 0;
                        double range = new Vector2(point[0] - odometry.odometryBufferForTuret.read().getPosition().getX(), point[1] - odometry.odometryBufferForTuret.read().getPosition().getY()).length();

                        double theta;

                        //в градусах
                        if (range >= 290) theta = 46;
                        else if (range >= 150) theta = 55;
                        else if (range >= 70) theta = 65;
                        else theta = 70;

                        theta = Range.clip(theta, 46, 70);

                        targetSpeed = hoodedShoter.flyWheelClass.getTargetSpeed(theta, range);
                        curSpeed = hoodedShoter.flyWheelClass.flyWheelOdometry.odometryData.getHeadVel();

                        flyWheelPow = speedController.calculateVol(targetSpeed, curSpeed);

                        isFlyWheelReady = speedController.checkReadnees(targetSpeed, curSpeed);

                        double calclPos = hoodedShoter.angleController.getPos(theta);

                        hoodedShoter.angleController.execute(calclPos);

                        isAngleGrowUp = !hoodedShoter.angleController.getServo().isBusy();
                        switch (servoState){
                            case waiting:
                                if(isAngleGrowUp && isFlyWheelReady && cosB == 0){
                                    int index = 3 - hoodedShoter.digitalCellsClass.getArtifactCount();
                                    int neededColor = odometry.cameraClass.motif[index];
                                    hoodedShoter.digitalCellsClass.fire(neededColor);
                                    if (hoodedShoter.digitalCellsClass.isStopped) {
                                        servoState = ServoState.fired;
                                    }
                                    hoodedShoter.digitalCellsClass.setCurIterations(iterationCount);
                                    hoodedShoter.digitalCellsClass.update();
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
                    }
                    break;
                default:
                    break;
            }
        }else programState = ProgramState.Interrupted;

        checkButtons();

        headVoltage = Range.clip(cosB, -maxVol, maxVol);
        sinA = Range.clip(flyWheelPow, -maxVol, maxVol);
        collectorPow = Range.clip(collectorPow, -maxVol, maxVol);

        hoodedShoter.turretMotor.execute(headVoltage);
        hoodedShoter.flyWheelClass.execute(sinA);
        hoodedShoter.collector.execute(collectorPow);


    }

    @Override
    protected void showDataExt() {
        telemetry.addData("State", "fly %s angle %s vol %s",isFlyWheelReady, isAngleGrowUp, headVoltage);
//        telemetry.addData("motif", "%s %s %s",odometry.cameraClass.motif[0], odometry.cameraClass.motif[1], odometry.cameraClass.motif[2]);
//        telemetry.addData("inTerr", hoodedShoter.digitalCellsClass.isStopped);
//        joystickActivityClass.showData();
        hoodedShoter.showData();
        speedController.showData();
//        trackEmulator.showData();
//        pidfTunner.showData();
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
            super(0.0, 0,0,0.0, -1,1, mainFile);
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
            super(0.0, 0,0,0.25, -1, 1, mainFile);
        }

        private double returnDistance(double VelMax, double accel ){
            return Math.pow(VelMax, 2) / (2 * accel);
        }
        public double targHeadVel;
        public double errorHeading;
        public double targHead;
        public double calculateVol(OdometryData targetData, OdometryData currentData){
            Position2D targetPos = targetData.getPosition();
            Position2D currentPos = currentData.getPosition();

            // Находим ошибку положения
            targHead = targetPos.getHeading();
            double curHead = hoodedShoter.turretMotor.turretOdometry.localHead;
            double robotHead = odometry.odometryBufferForRobot.read().getPosition().getHeading();

            if (targHead - robotHead > Math.PI || targHead - robotHead < -Math.PI){
                targHead = getNorm(targHead - robotHead);
            }else targHead = targHead - robotHead;

            errorHeading = targHead - curHead;

            double distanceBreak = returnDistance(targetData.getHeadVel(), targetData.getHeadVel());

            targHeadVel = Math.signum(errorHeading) * Math.max(targetData.getHeadVel() * Math.min(1, Math.abs(errorHeading) / distanceBreak), MIN_TURRET_HEAD_SP);

            double pidHeadVel = calculate(targHeadVel, currentData.getHeadVel());

            if(Math.abs(errorHeading) < Math.toRadians(2)) {
                pidHeadVel = 0;
            }

            return pidHeadVel;
        }
        public double getNorm(double head){
            if (head > 0){
                head -= Math.PI * 2;
            }
            if (head < 0){
                head += Math.PI * 2;
            }
            return head;
        }
        @Override
        public void sayModuleName() {

        }
        @Override
        public void showData(){
            telemetry.addData("targH", targHead * RAD);
            telemetry.addData("targVel", targHeadVel * RAD);
            telemetry.addData("error", "%.2f", errorHeading * RAD);
            showDataExt();
        }
    }
    public class SpeedController extends Module {
        public PIDF pidfFlyWheel;
        public SpeedController(MainFile mainFile){
            super(mainFile);
            pidfFlyWheel = new PIDF(0, 0, 0,0.0018,-1, 1, mainFile);
        }

        @Override
        public void sayModuleName() {

        }

        @Override
        protected void showDataExt() {
            pidfFlyWheel.showData();
        }

        @Override
        protected void sayLastWords() {

        }

        public double calculateVol(double targetSpeed, double curSpeed){
            double pidPower = pidfFlyWheel.calculate(targetSpeed, curSpeed);

            return pidPower;
        }
        public boolean checkReadnees(double targetSpeed, double curSped){
            double errorPart = Math.abs(curSped / targetSpeed - 1);

            if (errorPart > 0.03)
            {
                return false;
            }
            else {
                return true;
            }
        }
    }
}

