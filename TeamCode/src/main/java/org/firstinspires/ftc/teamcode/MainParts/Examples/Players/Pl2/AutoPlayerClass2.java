package org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Pl2;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Joysticks.Extenders.Joystick1;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.PIDF;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.HoodedShooter;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.ProgramState;

public class AutoPlayerClass2 extends PlayerClass{
    public AutoPlayerClass2(RobotClass robotClass) {
        setJoystickActivityClass(new Joystick1(false));

        this.hoodedShooter = robotClass.hoodedShooter;
        this.odometry = robotClass.odometry;

        turretController = new TurretController();
        flyWheelController = new FlyWheelController();
        PDFTurner = new PDFTurner();
    }
    public HoodedShooter hoodedShooter;
    public Odometry odometry;

    public TurretController turretController;
    public PDFTurner PDFTurner;
    public FlyWheelController flyWheelController;
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
    private OdometryData targetData;
    private OdometryData turretCurrentData;
    private OdometryData robotCurrentData;

    @Override
    public void executeExt() {
        double maxVol = 0.8;

        double[] point = generalInformation.generalObjects.getPointVyr();

        turretCurrentData = odometry.odometryBufferForTuret.read();
        robotCurrentData = odometry.odometryBufferForRobot.read();

        Position2D targPos = new Position2D(point[0], point[1], 0);
        Position2D curPos = turretCurrentData.getPosition();

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
                joystickActivityClass.update(iterationCount, 1);
                collectorPow = 0;
                turretPow = 0;
                flyWheelPow = 0;
                break;
            case Main_loop:
                if (!isInterrupted){
                    if (generalInformation.programName == GeneralInformation.ProgramName.TeleOp) executeTeleOp();
                    else executeAuto();

                    checkButtons();
                }else {
                    turretPow = 0;
                    flyWheelPow = 0;
                    collectorPow = 0;
                    angleServoPos = hoodedShooter.angleController.getServo().servo.getPosition();
                    programState = ProgramState.Interrupted;
                }
                break;
        }

        turretPow = Range.clip(turretPow, -maxVol, maxVol);
        flyWheelPow = Range.clip(flyWheelPow, -maxVol, maxVol);
        collectorPow = Range.clip(collectorPow, -maxVol, maxVol);

        hoodedShooter.angleController.execute(angleServoPos);

        hoodedShooter.turretMotor.execute(turretPow);
        hoodedShooter.flyWheelClass.execute(flyWheelPow);
        hoodedShooter.collector.execute(collectorPow);

        hoodedShooter.update(iterationCount, 1);
    }

    @Override
    protected void executeTeleOp() {
        joystickActivityClass.update(iterationCount, 1);
        executeAuto();
    }

    @Override
    protected void executeAuto() {
        double range = new Vector2(targetData.getPosition().getX() - turretCurrentData.getPosition().getX(), targetData.getPosition().getY() - turretCurrentData.getPosition().getY()).length();

        double theta;

        //в градусах
        if (range >= 290) theta = 46;
        else if (range >= 150) theta = 55;
        else if (range >= 70) theta = 65;
        else theta = 70;

        theta = Range.clip(theta, 46, 70);

        double curSpeed = hoodedShooter.flyWheelClass.encodersClass.encodersBuffer.read().getHeadVel();
        double targetSpeed = 0;

        int count = hoodedShooter.digitalCellsClass.getArtifactCount();

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
                    angleServoPos = hoodedShooter.angleController.getPos(theta);
                    targetSpeed = hoodedShooter.flyWheelClass.getTargetSpeed(theta, range);

                    isFlyWheelReady = flyWheelController.checkReadnees(targetSpeed, curSpeed);
                    isAngleGrowUp = !hoodedShooter.angleController.getServo().isBusy();
                    switch (servoState){
                        case waiting:
                            if(isAngleGrowUp && isFlyWheelReady && turretPow == 0){
                                int index = 3 - count;
                                int neededColor = odometry.cameraClass.motif[index];
                                hoodedShooter.digitalCellsClass.fire(neededColor);
                                if (hoodedShooter.digitalCellsClass.isStopped) {
                                    servoState = ServoState.fired;
                                }
                            }
                            break;
                        case fired:
                            if(!hoodedShooter.digitalCellsClass.triggeredServo.isBusy()) {
                                hoodedShooter.digitalCellsClass.prepareServo();
                                servoState = ServoState.prepared;
                            }
                            break;
                        case prepared:
                            if(!hoodedShooter.digitalCellsClass.triggeredServo.isBusy()) {
                                hoodedShooter.digitalCellsClass.isStopped = false;
                                servoState = ServoState.waiting;
                            }
                            break;
                    }
                }

                break;
            default:
                break;
        }

        OdometryData calculatedData = flyWheelController.calculateVol(targetSpeed, curSpeed);
        flyWheelPow = calculatedData.getHeadVel();
    }

    @Override
    protected void showDataExt() {
        joystickActivityClass.showData();
        hoodedShooter.showData();
        flyWheelController.showData();
        turretController.showData();
        PDFTurner.showData();
    }

    @Override
    public void buttonAReleased() {
        //Есть реализация у 1 игрока
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
        OdometryData calculatedData = turretController.calculateVol(targetData, turretCurrentData);
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

    public class PDFTurner extends PIDF {
        private double[] stepSize = {1, 0.1, 0.01, 0.001, 0.0001, 0.00001};
        private int stepIndex;
        private int index;

        public PDFTurner() {
            super(0.0, 0,0,0.0, -1,1, "TestPid");
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
        protected void showDataExt() {
            telemetry.addLine(String.format("globalIndex: %s stepSize: %s", PDFTurner.index,  PDFTurner.stepSize[PDFTurner.stepIndex]));
        }
    }

    public class TurretController extends PIDF{
        public TurretController(){
            super(0.0, 0,0,0.3, -1, 1, "TurretPid");
        }

        private double returnDistance(double VelMax, double accel ){
            return Math.pow(VelMax, 2) / (2 * accel);
        }
        public OdometryData calculateVol(OdometryData targetData, OdometryData currentData){
            Position2D targetPos = targetData.getPosition();
            Position2D currentPos = currentData.getPosition();

            // Находим ошибку положения
            double targHead = targetPos.getHeading();
            double robotHead = robotCurrentData.getPosition().getHeading();
            double curHead = currentPos.getHeading() - robotHead;

            if (targHead - robotHead > Math.PI || targHead - robotHead < -Math.PI){
                targHead = getNorm(targHead - robotHead);
            }else targHead = targHead - robotHead;

            double errorHeading = targHead - curHead;

            double target_head = targetData.getHeadVel();
            double head_safe_brake = returnDistance(target_head, target_head);
            double headVel = Math.signum(errorHeading) * Math.max(target_head * Math.min(1, Math.abs(errorHeading) / head_safe_brake), MIN_TURRET_HEAD_SP);

            double pidHeadVel = calculate(headVel, currentData.getHeadVel());

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
    }
    public class FlyWheelController extends PIDF {
        public FlyWheelController(){
             super(0.05, 0, 0,0.0018,-1, 1, "FlyWheelPid");
        }

        public OdometryData calculateVol(double targetSpeed, double curSpeed){
            double pidPower = calculate(targetSpeed, curSpeed);

            return new OdometryData(new Vector2(0), pidPower);
        }
        public boolean checkReadnees(double targetSpeed, double curSpeed){
            double errorPart = Math.abs((curSpeed / targetSpeed) - 1);

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

