package org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Pl2;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Joysticks.Extenders.Joystick1;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.Reason;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.PIDF;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.HoodedShooter;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.ProgramState;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Wrappers.Examples.ServoMotorWrapper;

public class AutoPlayerClass2 extends PlayerClass{
    public AutoPlayerClass2(RobotClass robotClass) {
        setJoystickActivityClass(new Joystick1(false));

        this.hoodedShooter = robotClass.hoodedShooter;
        this.odometry = robotClass.odometry;

        turretController = new TurretController();
        flyWheelController = new FlyWheelController();
        pidfTurner = new PIDFTurner();

        trajectory1 = new OdometryData.DataBuilder()
                .createPath(new Position2D(0, 0,Math.toRadians(0)), MAX_ROBOT_LINEAR_SP, MAX_ROBOT_HEAD_SP, Reason.GoToCollect)
                .createPath(new Position2D(0, 0,Math.toRadians(180)), MAX_ROBOT_LINEAR_SP, MAX_ROBOT_HEAD_SP, Reason.GoToCollect);;
    }
    public HoodedShooter hoodedShooter;
    public Odometry odometry;

    public ServoMotorWrapper push0, push1, push2;

    private OdometryData.DataBuilder trajectory1;

    public TurretController turretController;
    public PIDFTurner pidfTurner;
    public FlyWheelController flyWheelController;
    public ServoState servoState = ServoState.waiting;
    public enum ServoState{
        waiting,
        firing
    }

    boolean isFlyWheelReady;
    boolean isAngleGrowUp;
    private double collectorPow;
    private double turretPow;
    private double flyWheelPow;
    private int count = 0;

    private double angleServoPos;
    private OdometryData targetData;
    private OdometryData turretCurrentData;
    private OdometryData robotCurrentData;

    @Override
    public void executeExt() {
        double maxVol = 0.9;

        double[] point = generalInformation.generalObjects.getPointVyr();

        pidfTurner.execute();

        turretCurrentData = odometry.bufferForTurret.read();
        robotCurrentData = odometry.bufferForRobot.read();

        Position2D targPos = new Position2D(point[0], point[1], 0);
        Position2D curPos = turretCurrentData.getPosition();

        Position2D deltaPos = targPos.minus(curPos);
        double targHead = new Position2D(0,0, Math.atan2(
                deltaPos.getY(),
                deltaPos.getX())).getHeading();
//        targetData = trajectory1.getData();
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
                odometry.cameraClass.coverUpCamera();
                if (!isInterrupted){
                    if (generalInformation.programName == GeneralInformation.ProgramName.TeleOp) executeTeleOp();
                    else executeAuto();

                    checkButtons();
                }else {
                    flyWheelPow = 0;
                    collectorPow = 0;

                    programState = ProgramState.Interrupted;
                }
                break;
        }

        turretPow = Range.clip(-turretPow, -maxVol, maxVol);
        flyWheelPow = Range.clip(flyWheelPow, -1, 1);
        collectorPow = Range.clip(collectorPow, -maxVol, maxVol);

        hoodedShooter.angleController.execute(angleServoPos);

        hoodedShooter.turretMotor.execute(turretPow);
        hoodedShooter.flyWheelClass.execute(flyWheelPow);
//        hoodedShooter.collector.execute(collectorPow);

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

        double curSpeed = hoodedShooter.flyWheelClass.flyWheelBuffer.read().getHeadVel();
        double targetSpeed = 0;

        int innerCount = hoodedShooter.digitalCellsClass.getArtifactCount();

        switch (generalInformation.gameTactick){
            case Load:
                targetSpeed = 0;

                if (innerCount != 3){
                    collectorPow = 1;
                }else {
                    count = 3;
                    collectorPow = 0;
                    programState = ProgramState.Finished;
                }
                break;
            case Fire:
                collectorPow = 0;

                if(innerCount == 0 && servoState == ServoState.waiting){
                    odometry.cameraClass.openUpCamera();
                    hoodedShooter.digitalCellsClass.continueUpdate();
                    programState = ProgramState.Finished;
                }else {
                    odometry.cameraClass.coverUpCamera();

                    angleServoPos = hoodedShooter.angleController.getPos(theta);
                    targetSpeed = hoodedShooter.flyWheelClass.getTargetSpeed(theta, range);

                    isFlyWheelReady = flyWheelController.checkReadnees(targetSpeed, curSpeed);
                    isAngleGrowUp = !hoodedShooter.angleController.getServo().isBusy(1);
                    if(count != 0){
                        switch (servoState){
                            case waiting:
                                if(isFlyWheelReady && (turretCurrentData.getPosition().getHeading() - targetData.getPosition().getHeading()) < ALLOWED_ZAZOR){
                                    int index = 3 - count;
                                    int neededColor = odometry.cameraClass.motif[index];
                                    hoodedShooter.digitalCellsClass.fire(neededColor);
                                    servoState = ServoState.firing;
                                }
                                break;
                            case firing:
                                if(!hoodedShooter.digitalCellsClass.triggeredServo.isBusy(16)) {
                                    hoodedShooter.digitalCellsClass.prepareServo();
                                    count = Math.max(count - 1, 0);
                                    servoState = ServoState.waiting;
                                }
                                break;
                        }
                    }else{
                        hoodedShooter.digitalCellsClass.prepareServo();
                        if(hoodedShooter.digitalCellsClass.isAllReady()) {
                            hoodedShooter.digitalCellsClass.continueUpdate();
                            hoodedShooter.digitalCellsClass.update(iterationCount, 1);
                            hoodedShooter.digitalCellsClass.stopUpdate();
                            innerCount = hoodedShooter.digitalCellsClass.getArtifactCount();
                            count = innerCount;
                        }
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
        pidfTurner.showData();
    }

    @Override
    public void buttonAReleased() {
        //Есть реализация у 1 игрока
//        push0.servo.setPosition(0.4);

    }
    @Override
    public void buttonAUnReleased() {
//        push0.servo.setPosition(0.1);
    }

    @Override
    public void buttonBReleased() {
        OdometryData calculatedData = turretController.calculateVol(targetData, turretCurrentData);
        turretPow = calculatedData.getHeadVel();
//
//        switch (joystickActivityClass.tDpadUpPressed % 3) {
//            case 0:
//                collectorPow = 0;
//                break;
//            case 1:
//                collectorPow = 1;
//                break;
//            case 2:
//                collectorPow = -1;
//                break;
//        }
    }

    @Override
    public void buttonBUnReleased() {
        joystickActivityClass.tDpadUpPressed = 0;

        OdometryData calculatedData = turretController.calculateVol(targetData, turretCurrentData);
        turretPow = calculatedData.getHeadVel();
    }

    @Override
    public void buttonXReleased() {
//        push1.servo.setPosition(0.4);
    }

    @Override
    public void buttonXUnReleased() {
//        push1.servo.setPosition(0.1);
    }

    @Override
    public void buttonYReleased() {
//        push2.servo.setPosition(0.4);
    }

    @Override
    public void buttonYUnReleased() {

    }

    public class PIDFTurner extends PIDF {
        private double[] stepSize = {1, 0.1, 0.01, 0.001, 0.0001, 0.00001};
        private int stepIndex;
        private int index;

        public PIDFTurner() {
            super(0.0, 0,1.0,0.35,-1,1, "TestPid");

        }
        double targetSpeed = 0;
        double targetHeadSpeed = 0;

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

            turretController.setPID(getkP(), getkI(), getkD(), getkF());
            targetHeadSpeed = Math.toRadians(20) * (joystickActivityClass.tBackPressed % 7);
        }
        @Override
        protected void showDataExt() {
            telemetry.addLine(String.format("globalIndex: %s stepSize: %s", pidfTurner.index,  pidfTurner.stepSize[pidfTurner.stepIndex]));
        }
    }

    public class TurretController extends PIDF{
        public TurretController(){
            super(0.07, 0,0.009,0.12, -1, 1, "TurretPid");
        }

        private double returnDistance(double VelMax, double accel){
            return Math.pow(VelMax, 2) / (2 * accel);
        }

        double lastRobotAngle;
        boolean flag = false;
        double futCur;
        double long_length;
        double short_length;
        double errorHeading;
        public OdometryData calculateVol(OdometryData targetData, OdometryData currentData){
            Position2D targetPos = targetData.getPosition();
            Position2D currentPos = currentData.getPosition();

            // Находим ошибку положения
            double targHeadG = targetPos.getHeading();
            double localHead = hoodedShooter.turretMotor.localHead;

            double length = targHeadG - currentPos.getHeading();
//
            double curRobotAngle = robotCurrentData.getPosition().getHeading();
//            double rawError = targHeadG - currentPos.getHeading();
//            double errorShort = normalizeAngle(rawError);
//            double errorLong = errorShort > 0 ? errorShort - 2*Math.PI : errorShort + 2*Math.PI;
//
//
//            double newPositionShort = localHead + errorShort;
//
//            double errorHeading;
//            if (newPositionShort > Math.toRadians(270) || newPositionShort < Math.toRadians(-90)) {
//                errorHeading = errorLong;
//            } else {
//                errorHeading = errorShort;
//            }

            if(!flag){
                if(Math.abs(length) > Math.PI) {long_length = length; short_length = getNorm(length);}
                else {long_length = getNorm(length); short_length = length;}

                futCur = localHead + short_length;
                lastRobotAngle = curRobotAngle;
                flag = true;
            }else {
                if(Math.abs(curRobotAngle - lastRobotAngle) > Math.toRadians(1)){
                    flag = false;
                }
            }

            if(futCur > Math.toRadians(270) || futCur < Math.toRadians(-90)){
                errorHeading = getNorm(futCur) - localHead;
            }else errorHeading = futCur - localHead;

            double target_head_vel = targetData.getHeadVel();
            double head_safe_brake = returnDistance(target_head_vel, target_head_vel);

            double headVel = Math.signum(errorHeading) * Math.max(target_head_vel * Math.min(1, Math.abs(errorHeading) / head_safe_brake), MIN_TURRET_HEAD_SP);


            if(Math.abs(errorHeading) < ALLOWED_ZAZOR) {
                headVel = 0;
                resetI();
            }

            double pidHeadVel = calculate(headVel, currentData.getHeadVel(), 1);

            return new OdometryData(new Vector2(0), pidHeadVel);
        }

        @Override
        public void showData() {
            telemetry.addData("error", errorHeading * RAD);
            telemetry.addData("local", hoodedShooter.turretMotor.localHead * RAD);
            telemetry.addData("futurCurent", futCur * RAD);
            this.showDataExt();
        }

        public double getDelta(double head){
            if (head > Math.PI){
                head -= Math.PI;
            }
            if (head < -Math.PI){
                head += Math.PI;
            }
            return head;
        }
        public double getNorm(double head){
            if (head > Math.PI){
                head -= Math.PI * 2;
            }
            if (head < -Math.PI){
                head += Math.PI * 2;
            }
            return head;
        }
    }
    public class FlyWheelController extends PIDF {
        public FlyWheelController(){
             super(0.003, 0, 0,0.00178,-1, 1, "FlyWheelPid");
        }

        public OdometryData calculateVol(double targetSpeed, double curSpeed){
            double pidPower = calculate(targetSpeed, curSpeed, 1);

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

