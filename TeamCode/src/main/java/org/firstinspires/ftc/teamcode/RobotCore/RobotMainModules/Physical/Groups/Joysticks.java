package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Groups;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;

public class Joysticks implements Module {
    private final OpMode op;

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public int gear = 1, dpadUp, bPressed = 1, gearTele = 0, lastGearTele = 0;

    private double nowPos;
    private boolean isUpGear;

    private boolean isDpadUpReleased = false;
    public boolean isAY_g1 = false, isX_g1 = false, isX_g2 = true, isA_g2 = false, isB_g2 = false, isRBum_g1 = false, isLBum_g1 = false, isY_g2 = false, isBack_g2 = true;

    private boolean switchAY_g1 = false, switchX_g1 = false, switchX_g2 = false,
            switchA_g2 = false, switchB_g2 = false, switchRBum_g1 = false, switchLBum_g1 = false, switchY_g2 = false,
    rightTrigger = false, leftTrigger = false, switchBack_g2 = false;

    public Joysticks(OpMode op){
        this.op = op;
    }

    @Override
    public void init() {
        this.gamepad1 = op.gamepad1;
        this.gamepad2 = op.gamepad2;

        op.telemetry.addLine("Gamepads Inited");
    }

    public synchronized Gamepad getGamepad1() {
        return gamepad1;
    }

    public synchronized Gamepad getGamepad2() {
        return gamepad2;
    }

    public boolean isB_g2() {
        return isB_g2;
    }

    public boolean isY_g2() {
        return isY_g2;
    }

    public synchronized boolean isBack_G2(){
        if(gamepad2.back && !switchBack_g2){
            isBack_g2 = !isBack_g2;
            switchBack_g2 = true;
        }
        if(!gamepad2.back && switchBack_g2){
            switchBack_g2 = false;
        }
        return isBack_g2;
    }
    public synchronized boolean isAandY_G1(){

        if(gamepad1.a && gamepad1.y && !switchAY_g1) {
            isAY_g1 = !isAY_g1;
            switchAY_g1 = true;}

        if(!gamepad1.a && !gamepad1.y && switchAY_g1){
            switchAY_g1 = false;
        }

        return isAY_g1;
    }

    public synchronized boolean isB_G2(){
        if(bPressed == 4) bPressed = 1;

        if(gamepad2.b && !switchB_g2 && !gamepad2.start) {
            bPressed = Range.clip(bPressed + 1, 1,4);
            isB_g2 = !isB_g2;
            switchB_g2 = true;}
        if(!gamepad2.b && switchB_g2){
            switchB_g2 = false;
        }

        return isB_g2;
    }


    public int getBPressed() {
        return bPressed;
    }
    public int getGearTele(){
        if (gamepad2.right_trigger > 0.05 && !rightTrigger){
            isBack_g2 = false;
            gearTele = Range.clip(gearTele + 1, 0, 3);
            rightTrigger = true;
        }

        if(gamepad2.right_trigger < 0.05 && rightTrigger){
            rightTrigger = false;
        }

        if (gamepad2.left_trigger > 0.05 && !leftTrigger){
            isBack_g2 = false;
            gearTele = Range.clip(gearTele - 1, 0, 3);
            leftTrigger = true;
        }

        if(gamepad2.left_trigger < 0.05 && leftTrigger){
            leftTrigger = false;
        }

        return gearTele;
    }

    public int getDpadUp(boolean dpadDown){
        if(dpadDown){
            dpadUp = 0;
        }

        if(gamepad2.dpad_up && !isDpadUpReleased ) {
            dpadUp = Range.clip(dpadUp + 1, 0,5);
            isDpadUpReleased = true;}

        if(!gamepad2.dpad_up && isDpadUpReleased){
            isDpadUpReleased = false;
        }

        return dpadUp;
    }

    public synchronized boolean isX_G1() {

        if(gamepad1.x && !switchX_g1) {
            isX_g1 = !isX_g1;
            switchX_g1 = true;}

        if(!gamepad1.x && switchX_g1){
            switchX_g1 = false;
        }

        return isX_g1;
    }

    public synchronized boolean isX_G2(){

        if(gamepad2.x && !switchX_g2) {
            isX_g2 = !isX_g2;
            switchX_g2 = true;}

        if(!gamepad2.x && switchX_g2){
            switchX_g2 = false;
        }

        return isX_g2;
    }

    public synchronized boolean isA_G2(){

        if(gamepad2.a && !switchA_g2 && !gamepad2.start) {
            isA_g2 = !isA_g2;
            switchA_g2 = true;}

        if(!gamepad2.a && switchA_g2){
            switchA_g2 = false;
        }

        return isA_g2;
    }

    public synchronized boolean isRBum_G1(){

        if(gamepad1.right_bumper && !switchRBum_g1 ) {
            isRBum_g1 = !isRBum_g1;
            gear = Range.clip(gear - 1, 1,5);
            switchRBum_g1 = true;}

        if(!gamepad1.right_bumper && switchRBum_g1){
            switchRBum_g1 = false;
        }

        return isRBum_g1;
    }

    public synchronized boolean isY_G2(){

        if(gamepad2.y && !switchY_g2){
            isY_g2 = !isY_g2;
            switchY_g2 = true;
        }
        if (!gamepad2.y && switchY_g2){
            switchY_g2 = false;
        }
        return isY_g2;
    }

    public synchronized boolean isLBum_G1(){

        if(gamepad1.left_bumper && !switchLBum_g1 ) {
            isLBum_g1 = !isLBum_g1;
            gear = Range.clip(gear + 1, 1,5);
            switchLBum_g1 = true;}

        if(!gamepad1.left_bumper && switchLBum_g1){
            switchLBum_g1 = false;
        }

        return isLBum_g1;
    }

    public synchronized boolean isUpGear(){
        if(isLBum_G1()){
            isUpGear = true;
            isRBum_g1 = false;
        }

        if (isRBum_G1() ){
            isUpGear = false;
            isLBum_g1 = false;
        }
        return isUpGear;
    }



    public synchronized int getGear(){                       // создаем метод для получения передачи
        return gear;
    }

    public synchronized void checkJoysticksCombo(){
        op.telemetry.addLine("Joystick buttons statements")// выводим состояния кнопок
                .addData("\nisHookOpen", isA_g2)
                .addData("\nPropMode", isX_g2)
                .addData("\nisUpped", isB_g2);
        op.telemetry.addLine()
                .addData("\nisUpped", isY_g2);
    }

    public synchronized void checkGear(){
        op.telemetry.addData("\nGear", gear);              // выводим передачи
        op.telemetry.addLine();
    }

    public synchronized void getDpadUp(){
        op.telemetry.addData("\nDpadUp", dpadUp);
        op.telemetry.addLine();
    }

}
