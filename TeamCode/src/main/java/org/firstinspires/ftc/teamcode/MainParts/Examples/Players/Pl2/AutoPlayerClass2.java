package org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Pl2;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Joysticks.Extenders.Joystick1;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.GameTactick;
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

    private OdometryData.DataBuilder trajectory1;
    double range;
    public TurretController turretController;
    public PIDFTurner pidfTurner;
    public FlyWheelController flyWheelController;
    public ServoState servoState = ServoState.waiting;
    public enum ServoState{
        waiting,
        firing,
        prep
    }

    boolean isFlyWheelReady;
    boolean isAngleGrowUp;
    private double collectorPow;
    private double turretPow;
    private double flyWheelPow;
    private int count = 0;

    private int iteratNum = 0;
    private double angleServoPos;
    private OdometryData targetData;
    private OdometryData turretCurrentData;
    private OdometryData robotCurrentData;

    @Override
    public void executeExt() {
        double maxVol = 0.9;

        double[] point = generalInformation.generalObjects.getPointVyr();

//        pidfTurner.execute();

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
                if (!isInterrupted){
                    if (generalInformation.programName == GeneralInformation.ProgramName.TeleOp) executeTeleOp();
                    else executeAuto();

                    if(joystickActivityClass.right_trigger > 0.05){
                        joystickActivityClass.buttonB = false;
                        collectorPow = -1;
                    }
                    checkButtons();
                }else {
                    collectorPow = 0;

                    programState = ProgramState.Interrupted;
                }
                break;
        }

//        if(generalInformation.programName == GeneralInformation.ProgramName.Auto){
//            turretPow = 0;
//        }
        turretPow = Range.clip(-turretPow, -maxVol, maxVol);
        flyWheelPow = Range.clip(flyWheelPow, -1, 1);
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

    double targetSpeed;
    @Override
    protected void
    executeAuto() {

        if(odometry.cameraClass.decisionMargin > 0 && odometry.cameraClass.id == generalInformation.generalObjects.aprilTagIds ){
            range = odometry.cameraClass.range;
        }else range = new Vector2(targetData.getPosition().getX() - turretCurrentData.getPosition().getX(), targetData.getPosition().getY() - turretCurrentData.getPosition().getY()).length();

        double theta;
        //в градусах
        if (range >= 230) theta = 46;
        else if (range >= 150) theta = 55;
        else if (range >= 70) theta = 65;
        else theta = 70;

        theta = Range.clip(theta, 46, 70);

        double curSpeed = hoodedShooter.flyWheelClass.flyWheelBuffer.read().getHeadVel();
        targetSpeed = 0;

        int innerCount = hoodedShooter.digitalCellsClass.getArtifactCount();

        switch (generalInformation.gameTactick){
            case Load:
                iteratNum = 0;
                targetSpeed = 0;

                collectorPow = 1;

//                odometry.cameraClass.openUpCamera();
                count = 3;
                hoodedShooter.digitalCellsClass.prepareServo(0);
                hoodedShooter.digitalCellsClass.prepareServo(1);
                hoodedShooter.digitalCellsClass.prepareServo(2);
                servoState = ServoState.waiting;
                break;
            case Fire:
//                odometry.cameraClass.coverUpCamera();
                collectorPow = 0;

                angleServoPos = hoodedShooter.angleController.getPos(theta);
                targetSpeed = hoodedShooter.flyWheelClass.getTargetSpeed(theta, range);

                isFlyWheelReady = flyWheelController.checkReadnees(targetSpeed, curSpeed);
                isAngleGrowUp = !hoodedShooter.angleController.getServo().isBusy(1);
                boolean isTurretReady = (turretCurrentData.getPosition().getHeading() - targetData.getPosition().getHeading()) < ALLOWED_ZAZOR;
                if(count != 0){
                    switch (servoState){
                        case waiting:
                            if(isFlyWheelReady && isTurretReady){
                                int index = count - 1;
//                                int neededColor = odometry.cameraClass.motif[index];
                                hoodedShooter.digitalCellsClass.fireCell(index);
                                servoState = ServoState.firing;
                            }
                            break;
                        case firing:
                            if(!hoodedShooter.digitalCellsClass.triggeredServo.isBusy(2.2)) {
                                int index = count - 1;

                                hoodedShooter.digitalCellsClass.prefirePos(index);

                                servoState = ServoState.prep;
                            }
                            break;
                        case prep:
                            if(!hoodedShooter.digitalCellsClass.triggeredServo.isBusy(1)){
                                count = Math.max(count - 1, 0);
                                servoState = ServoState.waiting;
                            }
                            break;
                    }
                }else{
                    iteratNum++;
                    count = 3;
                    hoodedShooter.digitalCellsClass.prepareServo(0);
                    hoodedShooter.digitalCellsClass.prepareServo(1);
                    hoodedShooter.digitalCellsClass.prepareServo(2);
                }
                if(iteratNum == 3) {programState = ProgramState.Finished;}
                break;
            default:
                break;
        }

        targetSpeed = Range.clip(targetSpeed, 0, 628);

        OdometryData calculatedData = flyWheelController.calculateVol(targetSpeed, curSpeed);
        flyWheelPow = calculatedData.getHeadVel();
    }

    @Override
    protected void showDataExt() {
        telemetry.addData("range", range);
        telemetry.addData("speed", targetSpeed);
        telemetry.addData("flyPow",flyWheelPow);
        telemetry.addData("states", "fly %s turret %s", isFlyWheelReady , (turretCurrentData.getPosition().getHeading() - targetData.getPosition().getHeading()) < ALLOWED_ZAZOR);
        telemetry.addData("servoState", servoState.name());
//        joystickActivityClass.showData();
//        hoodedShooter.showData();
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

        switch (joystickActivityClass.tDpadUpPressed % 2) {
            case 0:
                collectorPow = 0;
                break;
            case 1:
                collectorPow = -1;
                break;
        }
        generalInformation.gameTactick = GameTactick.Fire;
    }

    @Override
    public void buttonBUnReleased() {
        joystickActivityClass.tDpadUpPressed = 0;

        OdometryData calculatedData = turretController.calculateVol(targetData, turretCurrentData);
        turretPow = calculatedData.getHeadVel();
        generalInformation.gameTactick = GameTactick.Load;
    }

    @Override
    public void buttonXReleased() {
        turretPow = joystickActivityClass.sinB;
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
            super(0.0, 0,0.0,0.0,-1,1, "TestPid");
            setPID(flyWheelController);

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

            flyWheelController.setPID(getkP(), getkI(), getkD(), getkF());
            targetHeadSpeed = Math.toRadians(20) * (joystickActivityClass.tBackPressed % 7);
        }
        @Override
        protected void showDataExt() {
            telemetry.addLine(String.format("globalIndex: %s stepSize: %s", pidfTurner.index,  pidfTurner.stepSize[pidfTurner.stepIndex]));
        }
    }

    public class TurretController extends PIDF{
        public TurretController(){
            super(0.05, 0,0.05,0.25, -1, 1, "TurretPid");
        }

        private double returnDistance(double VelMax, double accel){
            return Math.pow(VelMax, 2) / (2 * accel);
        }

        public OdometryData calculateVol(OdometryData targetData, OdometryData currentData){
            Position2D targetPos = targetData.getPosition();
            Position2D currentPos = currentData.getPosition();

            // Находим ошибку положения
            double targHeadG = targetPos.getHeading();
            double robotHeadG = robotCurrentData.getPosition().getHeading();
            double targHeadL = targHeadG - robotHeadG;
            double localHead = hoodedShooter.turretMotor.localHead;

            if (targHeadL > Math.PI || targHeadL < -Math.PI){
                targHeadL = getNorm(targHeadL);
            }

            double target_head_vel = targetData.getHeadVel();
            double head_safe_brake = returnDistance(target_head_vel, target_head_vel);
            double errorHeading;

            if(targHeadL > Math.PI || targHeadL < -Math.PI){
                errorHeading = targHeadL - localHead;
            }
            else {
                if(odometry.cameraClass.decisionMargin > 0 && odometry.cameraClass.id == generalInformation.generalObjects.aprilTagIds ){
                    errorHeading = odometry.cameraClass.bearing;
                }
                else errorHeading = targHeadL - localHead;
            }
            double headVel = Math.signum(errorHeading) * Math.max(target_head_vel * Math.min(1, Math.abs(errorHeading) / head_safe_brake), MIN_TURRET_HEAD_SP);


            if(Math.abs(errorHeading) < ALLOWED_ZAZOR) {
                headVel = 0;
                resetI();
                setPID(0.05, 0,0.05,0.25);
            }

            if(Math.abs(headVel - currentData.getHeadVel()) < Math.abs(headVel) && headVel != 0){
                setPID(0.05, 0,0.05,getkF() * 1.02);
            }
            double pidHeadVel = calculate(headVel, currentData.getHeadVel(), 1);

            return new OdometryData(new Vector2(0), pidHeadVel);
        }

        @Override
        public void showData() {
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
             super(0.005, 0, 0,0.00185,-1, 1, "FlyWheelPid");
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

