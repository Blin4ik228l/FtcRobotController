package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MessageTelemetry implements Module {
    public Telemetry telemetry;

    private OpMode op;
    private Odometry odometry;
    private MecanumDrivetrain drivetrain;
    private TeleSkope teleSkope;

    boolean isTelemetryKilled = false;
    boolean isDrive = false;
    boolean isTicksToCm = false;

    boolean pastState, switchable = false;

    private double targetVelX, targetVelY, targetAngleVel;
    private String nameOfValueForAngular, nameOfValueForNonAngular;

    private double forwardVol, sideVol, angleVol;
    private double kForLine, kForRound;

    //TODO реализовать телеметрию
    public MessageTelemetry(OpMode op, Odometry odometry, MecanumDrivetrain drivetrain, TeleSkope teleSkope){
        this.op = op;
        this.odometry = odometry;
        this.drivetrain = drivetrain;
        this.teleSkope = teleSkope;

        telemetry = op.telemetry;
    }

    @Override
    public void init() {
        startTelemetry();
    }

    private void startTelemetry(){
        while (!isTelemetryKilled) {
            if (isDrive) {
                showGlobalVel();
                showEncodersVel();
                showTargetVelGamepads();
                showTargetVoltageGamepads();
            }

            showMotorsDriveTrainVoltage();

            if(isTicksToCm){
                showEncodersCM();
            }else {
                showEncodersTicks();}

            showGlobalPos();

            switchTicksToCmState();
            updateRobotMovingState();

            upDateTelemetry();
        }
    }

    public void setTargetVel(double targetVelX, double targetVelY,
                             double targetAngleVel, String nameOfValueForNonAngular, String nameOfValueForAngular){
        this.targetVelX = targetVelX;
        this.targetVelY = targetVelY;
        this.targetAngleVel = targetAngleVel;
        this.nameOfValueForNonAngular = nameOfValueForNonAngular;
        this.nameOfValueForAngular = nameOfValueForAngular;
    }

    public void setTargetVoltage(double forwardVol, double sideVol, double angleVol){
        this.forwardVol = forwardVol;
        this.sideVol = sideVol;
        this.angleVol = angleVol;
    }

    public void setKoefForDrives(double kForLine, double kForRound){
        this.kForLine = kForLine;
        this.kForRound = kForRound;
    }

    public void addData(String nameofValue, double number){
        telemetry.addData(nameofValue, number);
    }

    public void addData(String nameofValue, int number){
        telemetry.addData(nameofValue, number);
    }

    public void addData(String nameofValue, String string){
        telemetry.addData(nameofValue, string);
    }

    private void showTargetVoltage(){
        telemetry.addLine("Target voltage");
        telemetry.addData("Напряжение для движения по Y:", forwardVol);
        telemetry.addData("Напряжение для движения по X:", sideVol);
        telemetry.addData("Напряжение для кручения:", angleVol);
        telemetry.addLine();
    }
    private void showTargetVoltageGamepads(){
        telemetry.addLine("Target voltage from gamepads");
        telemetry.addData("Напряжение робота для Y по джойстику:", targetVelX * kForLine);
        telemetry.addData("Напряжение робота для X по джойстику:", targetVelY * kForLine);
        telemetry.addData("Напряжение угловой скорости для робота по джойстику:", targetAngleVel * kForRound);
        telemetry.addLine();
    }

    private void showTargetVelGamepads(){
        telemetry.addLine("Target vel from gamepads");
        telemetry.addData("Скорость робота по Y по джойстику "+ nameOfValueForNonAngular + ":", targetVelX);
        telemetry.addData("Скорость робота по X по джойстику" + nameOfValueForNonAngular + ":", targetVelY);
        telemetry.addData("Угловая скорость робота по джойстику" + nameOfValueForAngular + ":", targetAngleVel);
        telemetry.addLine();
    }

    private void showEncodersTicks(){
        telemetry.addLine("Encoders ticks");
        telemetry.addData("encL ticks:", odometry.encL.getCurrentPosition());
        telemetry.addData("encM ticks:", odometry.encM.getCurrentPosition());
        telemetry.addData("encR ticks:", odometry.encL.getVelocity());
        telemetry.addLine();
    }

    private void showEncodersCM(){
        telemetry.addLine("Encoders cm");
        telemetry.addData("encL cm:", odometry.ticksToCm(odometry.encL.getCurrentPosition()));
        telemetry.addData("encM cm:", odometry.ticksToCm(odometry.encM.getCurrentPosition()));
        telemetry.addData("encR cm:", odometry.ticksToCm(odometry.encR.getCurrentPosition()));
        telemetry.addLine();
    }

    private void showEncodersVel(){
        telemetry.addLine("Encoders velocity");
        telemetry.addData("encL velocity:", odometry.encL.getVelocity());
        telemetry.addData("encM velocity:", odometry.encM.getVelocity());
        telemetry.addData("encR velocity:", odometry.encL.getVelocity());
        telemetry.addLine();
    }

    private void showGlobalPos(){
        telemetry.addLine("Global position");
        telemetry.addData("GlobalX:", odometry.getGlobalPosition().x);
        telemetry.addData("GlobalY:", odometry.getGlobalPosition().y);
        telemetry.addData("GlobalHeading:", odometry.getGlobalPosition().heading);
        telemetry.addLine();
    }

    private void showGlobalVel(){
        telemetry.addLine("Robot velocity");
        telemetry.addData("VelocityX:", odometry.getVelocity().x);
        telemetry.addData("VelocutyY:", odometry.getGlobalPosition().y);
        telemetry.addData("AngularVel:", odometry.getAngularVelocity());
        telemetry.addLine();
    }

    private void showMotorsDriveTrainVoltage(){
        telemetry.addLine("Voltage drivetrain");
        telemetry.addData("leftF voltage:", drivetrain.leftF.getPower());
        telemetry.addData("rightF voltage:", drivetrain.rightF.getPower());
        telemetry.addData("leftB voltage:", drivetrain.leftB.getPower());
        telemetry.addData("rightB voltage:", drivetrain.rightB.getPower());
        telemetry.addLine();
    }


    private void updateRobotMovingState(){
        isDrive = op.gamepad1.left_stick_y != 0 || op.gamepad1.left_stick_x != 0 || op.gamepad1.right_stick_x != 0;
    }

    private void switchTicksToCmState(){

        if((op.gamepad1.a && op.gamepad2.y) == !pastState){
            switchable = !switchable;
        }

        pastState = (op.gamepad1.a && op.gamepad2.y);

        isTicksToCm = switchable;
    }

    private void upDateTelemetry(){
        telemetry.update();
    }

    public void killTelemetry(){
        isTelemetryKilled = true;
    }

}
