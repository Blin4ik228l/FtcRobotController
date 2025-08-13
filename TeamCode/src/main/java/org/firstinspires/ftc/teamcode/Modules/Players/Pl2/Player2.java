package org.firstinspires.ftc.teamcode.Modules.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.Modules.Players.Player;

public class Player2 extends Player implements Runnable {
   public Player2(Gamepad gamepad, TeleSkope teleSkope, OpMode op){
       super(op.telemetry);

        playersGamepad = gamepad;
        this.teleSkope = teleSkope;
        joystickActivity = new JoystickActivity();
    }
    public TeleSkope teleSkope;
    public JoystickActivity joystickActivity;

    @Override
    public void run() {
        play();
    }

    @Override
    public void play() {
        joystickActivity.checkActivity();

        double leftStickY = -playersGamepad.left_stick_y;

        double upStandingVel = -playersGamepad.right_stick_y;

        double horizontalPos = 0;
        double targetHeight = 0;
        double hookPos = 0;
        double flipPos = 0;

        if(joystickActivity.tDpadDownPressed == 1){
            joystickActivity.tDpadDownPressed = 0;
            joystickActivity.tDpadUpPressed = 0;
        }

        switch (joystickActivity.tDpadUpPressed){
            case 0:
                horizontalPos = ConstsTeleskope.CLOSE_POS_HORIZONTAL2;
                break;

            case 1:
                horizontalPos = ConstsTeleskope.CLOSE_POS_HORIZONTAL2 + 0.05;
                break;

            case 2:
                horizontalPos = ConstsTeleskope.CLOSE_POS_HORIZONTAL2 + 0.1;
                break;

            case 3:
                horizontalPos = ConstsTeleskope.CLOSE_POS_HORIZONTAL2 + 0.15;
                break;

            case 4:
                horizontalPos = ConstsTeleskope.CLOSE_POS_HORIZONTAL2 + 0.2;
                break;

            case 5:
                horizontalPos = ConstsTeleskope.OPEN_POS_HORIZONTAL2;
                break;
        }

        if(joystickActivity.tRightTriggerPressed == 1){
            joystickActivity.tRightTriggerPressed = 0;
            joystickActivity.tLeftTriggerPressed = 0;
        }

        //Поднастроить
        switch (joystickActivity.tLeftTriggerPressed) {
            case 0:
                targetHeight = 0;
                break;

            case 1:
                targetHeight = 17;
                break;

            case 2:
                targetHeight = 34;
                break;
        }

        if(joystickActivity.buttonY) {
            flipPos = HANG_POS_FLIP;
            joystickActivity.buttonB = false;
        }

        if (joystickActivity.buttonB ) {
            flipPos = TAKE_POS_FLIP;
            joystickActivity.buttonY = false;
        }

        if (!joystickActivity.buttonY && !joystickActivity.buttonY)
        {
            flipPos = MIDLE_POS_FLIP;
        }

        if (joystickActivity.buttonA){
            hookPos = CLOSE_POS_HOOK;
        }else{
            hookPos = OPEN_POS_HOOK;
        }

        if(playersGamepad.x) {
            upStandingVel = -0.7;
        }else{
            hookPos = OPEN_POS_HOOK;
            targetHeight = 17;
            joystickActivity.tLeftTriggerPressed = 1;
        }

        teleSkope.setTeleskope(upStandingVel, false, targetHeight, horizontalPos, hookPos, flipPos);

        showData();
    }

    @Override
    public void showData() {
        teleSkope.lift.selfData.showHeight();
        teleSkope.servos.showServosPos();
    }
}
