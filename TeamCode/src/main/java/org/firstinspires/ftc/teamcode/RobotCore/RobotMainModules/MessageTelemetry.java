package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;

import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.atomic.AtomicInteger;

public class MessageTelemetry  implements Module {

    public final Telemetry telemetry;

    public final Robot robot;
    private final OpMode op;

    boolean isTelemetryKilled = false;
    boolean isRobotDrive = false;
    boolean isTicksToCm = false;

    boolean pastState, switchable = false;
    int released = 0;

    private double forwardVol, sideVol, angleVol;

    //TODO реализовать телеметрию
    public MessageTelemetry(OpMode op, Robot robot){
        this.robot = robot;
        this.op = op;

        telemetry = op.telemetry;
    }

    @Override
    public void init() {
        if(telemetry != null){
        telemetry.addLine("Telemetry is Ready!");
        }

    }

    public void dataForAuto(){
        showGlobalPos();
        showGlobalVelLength();
        showMotorsDriveTrainVoltage();
    }

    public void dataForTeleOp(){
//            if (isRobotDrive) {
//
//                showEncodersVel();
////                showTargetVelGamepads();
////                showTargetVoltageGamepads();
//            }
            showGlobalVelLength();
            showMotorsDriveTrainVoltage();
//            telemetry.addData("time", robot.odometry.dt);
//            showMaxVelLength();
//            showMaxAccelLength();
//
//            if (isTicksToCm) {
//                showEncodersCM();
//            } else {
//                showEncodersTicks();
//            }
//
//            showGlobalPos();
//
//            switchTicksToCmState();
//            updateRobotMovingState();
    }

//    public void setTargetVel(double targetVelX, double targetVelY,
//                             double targetAngleVel, String nameOfValueForNonAngular, String nameOfValueForAngular){
//        this.targetVelX = targetVelX;
//        this.targetVelY = targetVelY;
//        this.targetAngleVel = targetAngleVel;
//        this.nameOfValueForNonAngular = nameOfValueForNonAngular;
//        this.nameOfValueForAngular = nameOfValueForAngular;
//    }

    public void showMaxAccel(){
        telemetry.addLine("Max accel");
        telemetry.addData("Accelx:", robot.odometry.getMaxAcceleration().x);
        telemetry.addData("Accely:", robot.odometry.getMaxAcceleration().y);
        telemetry.addLine();
    }

    public void showMaxAccelLength(){
        telemetry.addLine("Max accel");
        telemetry.addData("AccelLength:", robot.odometry.getMaxAcceleration().length());
        telemetry.addLine();
    }

    public void showMaxVelLength(){
        telemetry.addLine("Max vel");
        telemetry.addData("VelLength:", robot.odometry.getMaxVel().length());
        telemetry.addLine();
    }

    public void setTargetVoltage(double forwardVol, double sideVol, double angleVol){
        this.forwardVol = forwardVol;
        this.sideVol = sideVol;
        this.angleVol = angleVol;
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

    private void showTasks(){
        telemetry.addLine("Задачи");
        telemetry.addData("Задача:", robot.taskManager.getTaskDeque().isEmpty());
        telemetry.addLine();
    }
    private void showCompletedTasks(){
        telemetry.addLine("Выполненые задачи");
        telemetry.addData("Задача:", robot.taskManager.getCompletedTasks().isEmpty());
        telemetry.addLine();
    }
    private void showExecutingTasks(){
        telemetry.addLine("Задачи в обработке");
        telemetry.addData("Задача:", robot.taskManager.getExecutingDeque().isEmpty());
        telemetry.addLine();
    }
    private void showTargetVoltage(){
        telemetry.addLine("Target voltage");
        telemetry.addData("Напряжение для движения по Y:", forwardVol);
        telemetry.addData("Напряжение для движения по X:", sideVol);
        telemetry.addData("Напряжение для кручения:", angleVol);
        telemetry.addLine();

    }
//    private void showTargetVoltageGamepads(){
//        telemetry.addLine("Target voltage from gamepads");
//        telemetry.addData("Напряжение робота для Y по джойстику:", targetVelX * kForLine);
//        telemetry.addData("Напряжение робота для X по джойстику:", targetVelY * kForLine);
//        telemetry.addData("Напряжение угловой скорости для робота по джойстику:", targetAngleVel * kForRound);
//        telemetry.addLine();
//    }
//
//    private void showTargetVelGamepads(){
//        telemetry.addLine("Target vel from gamepads");
//        telemetry.addData("Скорость робота по Y по джойстику "+ nameOfValueForNonAngular + ":", targetVelX);
//        telemetry.addData("Скорость робота по X по джойстику" + nameOfValueForNonAngular + ":", targetVelY);
//        telemetry.addData("Угловая скорость робота по джойстику" + nameOfValueForAngular + ":", targetAngleVel);
//        telemetry.addLine();
//    }

    private void showEncodersTicks(){
        telemetry.addLine("Encoders ticks");
        telemetry.addData("encL ticks:", robot.odometry.encL.getCurrentPosition());
        telemetry.addData("encM ticks:", robot.odometry.encM.getCurrentPosition());
        telemetry.addData("encR ticks:", robot.odometry.encL.getCurrentPosition());
        telemetry.addLine();
    }

    private void showEncodersCM(){
        telemetry.addLine("Encoders cm");
        telemetry.addData("encL cm:", robot.odometry.ticksToCm(robot.odometry.encL.getCurrentPosition()));
        telemetry.addData("encM cm:", robot.odometry.ticksToCm(robot.odometry.encM.getCurrentPosition()));
        telemetry.addData("encR cm:", robot.odometry.ticksToCm(robot.odometry.encR.getCurrentPosition()));
        telemetry.addLine();

    }

    private void showEncodersVel(){
        telemetry.addLine("Encoders velocity");
        telemetry.addData("encL velocity:", robot.odometry.encL.getVelocity());
        telemetry.addData("encM velocity:", robot.odometry.encM.getVelocity());
        telemetry.addData("encR velocity:", robot.odometry.encL.getVelocity());
        telemetry.addLine();
    }

    private void showGlobalPos(){
        telemetry.addLine("Global position");
        telemetry.addData("GlobalX:", robot.odometry.getGlobalPosition().x);
        telemetry.addData("GlobalY:", robot.odometry.getGlobalPosition().y);
        telemetry.addData("GlobalHeading:", robot.odometry.getGlobalPosition().heading);
        telemetry.addLine();
    }

    private void showGlobalVelLength(){
        telemetry.addLine("Robot velocityLength");
        telemetry.addData("VelocityLength:", robot.odometry.getVelocity().length());
        telemetry.addData("AngularVel:", robot.odometry.getAngularVelocity());
        telemetry.addLine();
    }

    private void showGlobalVel(){
        telemetry.addLine("Robot velocity");
        telemetry.addData("VelocityX:", robot.odometry.getVelocity().x);
        telemetry.addData("VelocityY:", robot.odometry.getVelocity().y);
        telemetry.addData("AngularVel:", robot.odometry.getAngularVelocity());
        telemetry.addLine();
    }

    private void showMotorsDriveTrainVoltage(){
        telemetry.addLine("Voltage drivetrain");
        telemetry.addData("leftF voltage:", robot.drivetrain.leftF.getPower());
        telemetry.addData("rightF voltage:", robot.drivetrain.rightF.getPower());
        telemetry.addData("leftB voltage:", robot.drivetrain.leftB.getPower());
        telemetry.addData("rightB voltage:", robot.drivetrain.rightB.getPower());
        telemetry.addLine();
    }


    private void updateRobotMovingState(){
        isRobotDrive = op.gamepad1.left_stick_y != 0 || op.gamepad1.left_stick_x != 0 || op.gamepad1.right_stick_x != 0;
    }

    private void switchTicksToCmState(){

        if(op.gamepad1.a && op.gamepad1.y && released == 0) {
            switchable = !switchable;
            released = 1;}

        if(!op.gamepad1.a && !op.gamepad1.y && released != 0){
            released = 0;
        }

        isTicksToCm = switchable;
    }

    private void upDateTelemetry(){
        telemetry.update();
    }

    public void killTelemetry(){
        isTelemetryKilled = true;
    }


}
