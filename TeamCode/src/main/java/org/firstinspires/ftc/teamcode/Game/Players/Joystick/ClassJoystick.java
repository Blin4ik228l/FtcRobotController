package org.firstinspires.ftc.teamcode.Game.Players.Joystick;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ClassJoystick extends Thread implements Joystick {
    public ClassJoystick(Gamepad gamepad){
        this.gamepad = gamepad;
    }
    public Gamepad gamepad;

    public boolean isX_Button = false, isA_Button = false, isB_Button = false, isY_Button = false,
            isBack_Button = true, isStart_Button = false,
            isRightBumper = false, isLeftBumper = false,
            isRightTrigger = false, isLeftTrigger = false,
            isDpad_Up = false, isDpad_Down = false, isDpad_Left = false, isDpad_Right = false;

    public boolean switchA = false, switchX = false, switchB = false, switchY = false, switchBack = false, switchStart = false,
            switchRightBumper = false, switchLeftBumper = false,
            switchRightTrigger = false, switchLeftTrigger = false,
            switchDpad_Up = false, switchDpad_Down = false, switchDpad_Left = false, switchDpad_Right = false;

    public int tAPressed = 0, tBPressed = 0, tXPressed = 0, tYPressed = 0,
            tRightTriggerPressed = 0, tLeftTriggerPressed = 0, tRightBumperPressed = 0, tLeftBumperPressed = 0,
            tDpadDownPressed = 0, tDpadUpPressed = 0, tDpadLeftPressed = 0, tDpadRightPressed = 0,
            tBackPressed, tStartPressed = 0;

    @Override
    public boolean isA_Button() {
        switchButtonBool(gamepad.a, switchA, isA_Button, tAPressed);

        return isA_Button;
    }

    @Override
    public boolean isB_Button() {
        switchButtonBool(gamepad.b, switchB, isB_Button, tBPressed);

        return isB_Button;
    }

    @Override
    public boolean isX_Button() {
        switchButtonBool(gamepad.x, switchX, isX_Button, tXPressed);

        return isX_Button;
    }

    @Override
    public boolean isY_Button() {
        switchButtonBool(gamepad.y, switchY, isY_Button, tYPressed);

        return isY_Button;
    }

    @Override
    public boolean isRight_Trigger() {
        switchButtonNumb(gamepad.right_trigger, switchRightTrigger, isRightTrigger, tRightTriggerPressed);

        return isRightTrigger;
    }

    @Override
    public boolean isLeft_Trigger() {
        switchButtonNumb(gamepad.left_trigger, switchLeftTrigger, isLeftTrigger, tLeftTriggerPressed);

        return isLeftTrigger;
    }

    @Override
    public boolean isRight_Bumper() {
        switchButtonBool(gamepad.right_bumper, switchRightBumper, isRightBumper, tRightBumperPressed);

        return isRightBumper;
    }

    @Override
    public boolean isLeft_Bumper() {
        switchButtonBool(gamepad.left_bumper, switchLeftBumper, isLeftBumper, tLeftBumperPressed);

        return isLeftBumper;
    }

    @Override
    public boolean isDpad_Up() {
        switchButtonBool(gamepad.dpad_up, switchDpad_Up, isDpad_Up, tDpadUpPressed);

        return isDpad_Up;
    }

    @Override
    public boolean isDpad_Down() {
        switchButtonBool(gamepad.dpad_down, switchDpad_Down, isDpad_Down, tDpadDownPressed);

        return isDpad_Down;
    }

    @Override
    public boolean isDpad_Left() {
        switchButtonBool(gamepad.dpad_left, switchDpad_Left, isDpad_Left, tDpadLeftPressed);

        return isDpad_Left;
    }

    @Override
    public boolean isDpad_Right() {
        switchButtonBool(gamepad.dpad_right, switchDpad_Right, isDpad_Right, tDpadRightPressed);

        return isDpad_Right;
    }

    @Override
    public boolean isBack_Button() {
        switchButtonBool(gamepad.back, switchBack, isBack_Button, tBackPressed);

        return isBack_Button;
    }

    @Override
    public boolean isStart_Button() {
        switchButtonBool(gamepad.start, switchStart, isStart_Button, tStartPressed);

        return isStart_Button;
    }

    public void checkButtonsActivity(){
        isA_Button();
        isB_Button();
        isX_Button();
        isY_Button();

        isDpad_Up();
        isDpad_Down();
        isDpad_Left();
        isDpad_Right();

        isRight_Bumper();
        isLeft_Bumper();

        isRight_Trigger();
        isLeft_Trigger();

        isBack_Button();
        isStart_Button();
    }


    public void switchButtonBool(boolean buttonValue, boolean switchAble, boolean isButton, int countOfButton){
        if(buttonValue && !switchAble){
            isButton = !isButton;
            switchAble = true;
            countOfButton++;
        }
        if(!buttonValue && switchAble){
            switchAble = false;
        }
    }

    public void switchButtonNumb(double buttonValue, boolean switchAble, boolean isButton, int countOfButton){
        if(buttonValue > 0.05 && !switchAble){
            isButton = !isButton;
            switchAble = true;
            countOfButton++;
        }
        if(buttonValue < 0.05 && switchAble){
            switchAble = false;
        }
    }
}
