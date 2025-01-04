package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;

public class Joysticks implements Module {
    private final OpMode op;

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private boolean isAY_g1 = false, isX_g1 = false, isX_g2 = true, isA_g2 = false;

    private boolean switchAY_g1 = false, switchX_g1 = false, switchX_g2 = false, switchA_g2 = false ;

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


    public synchronized boolean isAandY_G1(){

        if(gamepad1.a && gamepad1.y && !switchAY_g1) {
            isAY_g1 = !isAY_g1;
            switchAY_g1 = true;}

        if(!gamepad1.a && !gamepad1.y && switchAY_g1){
            switchAY_g1 = false;
        }

        return isAY_g1;
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

        if(gamepad2.a && !switchA_g2) {
            isA_g2 = !isA_g2;
            switchA_g2 = true;}

        if(!gamepad2.a && switchA_g2){
            switchA_g2 = false;
        }

        return isA_g2;
    }

    public synchronized void checkJoysticksCombo(){
        isA_G2();
        isX_G2();
        isX_G1();
        isAandY_G1();
    }

}
