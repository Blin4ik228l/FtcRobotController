package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;

public class Joysticks implements Module {
    private final OpMode op;

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private boolean isHeadless = false, isCruise = false, isProp = true;

    private boolean switchH, switchC, switchP;

    public Joysticks(OpMode op){
        this.op = op;
    }

    @Override
    public void init() {
        this.gamepad1 = op.gamepad1;
        this.gamepad2 = op.gamepad2;
    }

    public Gamepad getGamepad1() {
        return gamepad1;
    }

    public Gamepad getGamepad2() {
        return gamepad2;
    }

    public boolean isCruiseDrive() {
        return isCruise;
    }

    public boolean isHeadlessDrive() {
        return isHeadless;
    }

    public boolean isProportionalTeleskope() {
        return isProp;
    }

    private void checkHeadless(){
        if(gamepad1.a && gamepad1.y && !switchH) {
            isHeadless = !isHeadless;
            switchH = true;}

        if(!gamepad1.a && !gamepad1.y && switchH){
            switchH = false;
        }
    }

    private void checkCruise() {
        if(gamepad1.x && !switchC) {
            isCruise = !isCruise;
            switchC = true;}
        if(!gamepad1.x && switchC){
            switchC = false;
        }
    }

    private void checkProport(){
        if(gamepad2.x && !switchP) {
            isProp = !isProp;
            switchP = true;}

        if(!gamepad2.x && switchP){
            switchP = false;
        }
    }

    public synchronized void checkJoysticksCombo(){
        checkCruise();
        checkHeadless();
        checkProport();
    }
}
