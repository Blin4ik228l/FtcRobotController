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

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.Enums.ProgramState;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

public class AutoPlayerClass2 extends PlayerClass{
    public AutoPlayerClass2(MainFile mainFile, RobotClass robotClass) {
        super(mainFile);
        setJoystickActivityClass(new Joystick1(mainFile));

        this.hoodedShoter = robotClass.hoodedShoter;
        this.odometry = robotClass.odometry;

        trackEmulator = new TrackEmulator(mainFile);
        flyWheelEmulator = new FlyWheelEmulator(mainFile);
        pidfTunner = new PIDFTunner(mainFile);
    }
    public HoodedShoter hoodedShoter;
    public Odometry odometry;

    public TrackEmulator trackEmulator;
    public PIDFTunner pidfTunner;
    public FlyWheelEmulator flyWheelEmulator;
    public ServoState servoState = ServoState.waiting;
    public enum ServoState{
        waiting,
        fired,
        prepared
    }

    boolean isFlyWheelReady;
    boolean isAngleGrowUp;
    private double collectorPow;
    private double turretPow;
    private double flyWheelPow;

    private double angleServoPos;
    OdometryData targetData;
    OdometryData currentData;

    @Override
    public void executeExt() {
        double maxVol = 0.8;

        double[] point = generalInformation.generalObjects.getPointVyr();

        currentData = new OdometryData(odometry.odometryBufferForTuret.read());

        Position2D targPos = new Position2D(point[0], point[1], 0);
        Position2D curPos = currentData.getPosition();

        Position2D deltaPos = targPos.minus(curPos);
        double targHead = new Position2D(0,0, Math.atan2(
                deltaPos.getY(),
                deltaPos.getX())).getHeading();
        targetData = new OdometryData(new Position2D(0,0, targHead), new Vector2(0), MAX_TURRET_HEAD_SP);

        //Выравниваем на ворота альянса
        switch (generalInformation.programStage){
            case Init:
                break;
            case Init_loop:
                collectorPow = 0;
                turretPow = 0;
                flyWheelPow = 0;
                hoodedShoter.digitalCellsClass.update();
                break;
            case Main_loop:
                if (!isInterrupted){
                    switch (generalInformation.programName){
                        case TeleOp:
                            executeTeleOp();
                            break;
                        default:
                            executeAuto();
                            break;
                    }
                    checkButtons();
                }else {
                    turretPow = 0;
                    flyWheelPow = 0;
                    collectorPow = 0;
                    angleServoPos = hoodedShoter.angleController.getServo().servo.getPosition();
                    programState = ProgramState.Interrupted;
                }
                break;
        }

        turretPow = Range.clip(turretPow, -maxVol, maxVol);
        flyWheelPow = Range.clip(flyWheelPow, -maxVol, maxVol);
        collectorPow = Range.clip(collectorPow, -maxVol, maxVol);

        hoodedShoter.angleController.execute(angleServoPos);

        hoodedShoter.turretMotor.execute(turretPow);
        hoodedShoter.flyWheelClass.execute(flyWheelPow);
        hoodedShoter.collector.execute(collectorPow);

        hoodedShoter.digitalCellsClass.setCurIterations(iterationCount);
        hoodedShoter.digitalCellsClass.update();
    }

    @Override
    protected void executeTeleOp() {
        joystickActivityClass.update();
        executeAuto();
    }

    @Override
    protected void executeAuto() {
        double range = new Vector2(targetData.getPosition().getX() - currentData.getPosition().getX(), targetData.getPosition().getY() - currentData.getPosition().getY()).length();

        double theta;

        //в градусах
        if (range >= 290) theta = 46;
        else if (range >= 150) theta = 55;
        else if (range >= 70) theta = 65;
        else theta = 70;

        theta = Range.clip(theta, 46, 70);

        double curSpeed = hoodedShoter.flyWheelClass.flyWheelOdometry.odometryData.getHeadVel();
        double targetSpeed = 0;

        int count = hoodedShoter.digitalCellsClass.getArtifactCount();

        switch (generalInformation.gameTactick){
            case Load:
                targetSpeed = 0;

                if (count != 3){
                    collectorPow = 1;
                }else {
                    collectorPow = 0;
                    programState = ProgramState.Finished;
                }
                break;
            case Fire:
                collectorPow = 0;

                if(count == 0 && servoState == ServoState.waiting){
                    programState = ProgramState.Finished;
                }else {
                    angleServoPos = hoodedShoter.angleController.getPos(theta);
                    targetSpeed = hoodedShoter.flyWheelClass.getTargetSpeed(theta, range);

                    isFlyWheelReady = flyWheelEmulator.checkReadnees(targetSpeed, curSpeed);
                    isAngleGrowUp = !hoodedShoter.angleController.getServo().isBusy();
                    switch (servoState){
                        case waiting:
                            if(isAngleGrowUp && isFlyWheelReady && turretPow == 0){
                                int index = 3 - count;
                                int neededColor = odometry.cameraClass.motif[index];
                                hoodedShoter.digitalCellsClass.fire(neededColor);
                                if (hoodedShoter.digitalCellsClass.isStopped) {
                                    servoState = ServoState.fired;
                                }
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

        OdometryData calculatedData = flyWheelEmulator.calculateVol(targetSpeed, curSpeed);
        flyWheelPow = calculatedData.getHeadVel();
    }

    @Override
    protected void showDataExt() {
        joystickActivityClass.showData();
        hoodedShoter.showData();
        flyWheelEmulator.showData();
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
        turretPow = joystickActivityClass.sinB;

        switch (joystickActivityClass.tDpadUpPressed % 3) {
            case 0:
                collectorPow = 0;
                break;
            case 1:
                collectorPow = 1;
                break;
            case 2:
                collectorPow = -1;
                break;
        }

    }

    @Override
    public void buttonBUnReleased() {
        OdometryData calculatedData = trackEmulator.calculateVol(targetData, currentData);
        turretPow = calculatedData.getHeadVel();
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

    public class TurretController{

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
            super(0.0, 0,0,0.3, -1, 1, mainFile);
        }

        private double returnDistance(double VelMax, double accel ){
            return Math.pow(VelMax, 2) / (2 * accel);
        }
        public double targHeadVel;
        public double errorHeading;
        public double targHead;

        public OdometryData calculateVol(OdometryData targetData, OdometryData currentData){
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

            if(Math.abs(errorHeading) < Math.toRadians(3)) {
                pidHeadVel = 0;
            }

            return new OdometryData(new Vector2(0), pidHeadVel);
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
    }
    public class FlyWheelEmulator extends Module {
        public PIDF pidfFlyWheel;
        public FlyWheelEmulator(MainFile mainFile){
            super(mainFile);
            pidfFlyWheel = new PIDF(0.05, 0, 0,0.0018,-1, 1, mainFile);
        }

        public OdometryData calculateVol(double targetSpeed, double curSpeed){
            double pidPower = pidfFlyWheel.calculate(targetSpeed, curSpeed);

            return new OdometryData(new Vector2(0), pidPower);
        }
        public boolean checkReadnees(double targetSpeed, double curSpeed){
            double errorPart = Math.abs(curSpeed / targetSpeed - 1);

            if (errorPart > 0.03)
            {
                return false;
            }
            else {
                return true;
            }
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

    }
}

