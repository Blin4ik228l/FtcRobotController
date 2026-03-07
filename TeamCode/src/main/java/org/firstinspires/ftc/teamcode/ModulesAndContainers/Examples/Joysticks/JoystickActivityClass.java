package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Joysticks;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableCollector;

public class JoystickActivityClass extends UpdatableCollector implements JoystickButtons {
    public Gamepad playersGamepad;
    public boolean buttonX = false, buttonA = false, buttonB = false, buttonY = false,
            buttonBack = false, buttonStart = false,
            bumperRight = false, bumperLeft = false,
            triggerRight = false, triggerLeft = false,
            dpad_Up = false, dpad_Down = false, dpad_Left = false, dpad_Right = false, a_and_Left_Trigger = false, touchpad = false;
    public boolean switchA = false, switchX = false, switchB = false, switchY = false, switchBack = false, switchStart = false,
            switchRightBumper = false, switchLeftBumper = false,
            switchRightTrigger = false, switchLeftTrigger = false,
            switchDpad_Up = false, switchDpad_Down = false, switchDpad_Left = false, switchDpad_Right = false, switchA_and_Left_Trigger = false , switchTouchpad = false;
    public int tAPressed = 0, tBPressed = 0, tXPressed = 0, tYPressed = 0,
            tRightTriggerPressed = 0, tLeftTriggerPressed = 0, tRightBumperPressed = 0, tLeftBumperPressed = 0,
            tDpadDownPressed = 0, tDpadUpPressed = 0, tDpadLeftPressed = 0, tDpadRightPressed = 0,
            tBackPressed, tStartPressed = 0, tA_and_Left_Trigger = 0, tTouchPressed = 0;
    public int iterNumA, iterNumB, iterNumX, iterNumY,
            iterNumRTrigger, iterNumLTrigger, iterNumRBumper, iterNumLBumper,
            iterNumDpadD, iterNumDpadU, iterNumDpadL, iterNumDpadR,
            iterNumBack, iterNumStart, iterNumA_and_LTrigger;

    public double cosA, sinA, cosB, sinB, right_trigger, left_trigger;

    public JoystickActivityClass(MainFile mainFile) {
        super(mainFile);
    }

    @Override
    protected void updateExt() {
        isA_Button();
        isB_Button();
        isX_Button();
        isY_Button();
        isRight_Trigger();
        isLeft_Trigger();
        isRight_Bumper();
        isLeft_Bumper();
        isDpad_Down();
        isDpad_Up();
        isDpad_Left();
        isDpad_Right();
        isBack_Button();
        isStart_Button();
        isTouchpad();

        cosA = playersGamepad.left_stick_x;
        sinA = -playersGamepad.left_stick_y;
        cosB = -playersGamepad.right_stick_x;
        sinB = -playersGamepad.right_stick_y;
        right_trigger = playersGamepad.right_trigger;
        left_trigger =  playersGamepad.left_trigger;
    }

    @Override
    public void isA_Button() {
        if(!playersGamepad.a && !switchA) return;

        if(playersGamepad.a && !switchA){
            buttonA = !buttonA;
            switchA = true;
            tAPressed++;
            iterNumA = iterationCount;
        }
        if(!playersGamepad.a && switchA){
            switchA = false;
        }
    }

    @Override
    public void isB_Button() {
        if(!playersGamepad.b && !switchB) return;

        if(playersGamepad.b && !switchB){
            buttonB = !buttonB;
            switchB = true;
            tBPressed++;
            iterNumB = iterationCount;
        }
        if(!playersGamepad.b && switchB){
            switchB = false;
        }
    }

    @Override
    public void isX_Button() {
        if(!playersGamepad.x && !switchX) return;

        if(playersGamepad.x && !switchX){
            buttonX = !buttonX;
            switchX = true;
            tXPressed++;
            iterNumX = iterationCount;
        }
        if(!playersGamepad.x && switchX){
            switchX = false;
        }
    }

    @Override
    public void isY_Button() {
        if(!playersGamepad.y && !switchY)return;

        if(playersGamepad.y && !switchY){
            buttonY = !buttonY;
            switchY = true;
            tYPressed++;
            iterNumY = iterationCount;
        }
        if(!playersGamepad.y && switchY){
            switchY = false;
        }
    }

    @Override
    public void isRight_Trigger() {
        if(playersGamepad.right_trigger < 0.05 && !switchRightTrigger) return;

        if(playersGamepad.right_trigger > 0.05 && !switchRightTrigger){
            triggerRight = !triggerRight;
            switchRightTrigger = true;
            tRightTriggerPressed++;
            iterNumRTrigger = iterationCount;
        }
        if(playersGamepad.right_trigger < 0.05 && switchRightTrigger){
            switchRightTrigger = false;
        }
    }

    @Override
    public void isLeft_Trigger() {
        if(playersGamepad.left_trigger < 0.05 && !switchLeftTrigger) return;

        if(playersGamepad.left_trigger > 0.05 && !switchLeftTrigger){
            triggerLeft = !triggerLeft;
            switchLeftTrigger = true;
            tLeftTriggerPressed++;
            iterNumLTrigger = iterationCount;
        }
        if(playersGamepad.left_trigger < 0.05 && switchLeftTrigger){
            switchLeftTrigger = false;
        }
    }

    @Override
    public void isRight_Bumper() {
        if(!playersGamepad.right_bumper && !switchRightBumper) return;

        if(playersGamepad.right_bumper && !switchRightBumper){
            bumperRight = !bumperRight;
            switchRightBumper = true;
            tRightBumperPressed++;
            iterNumRBumper = iterationCount;
        }
        if(!playersGamepad.right_bumper && switchRightBumper){
            switchRightBumper = false;
        }
    }

    @Override
    public void isLeft_Bumper() {
        if(!playersGamepad.left_bumper && !switchLeftBumper) return;

        if(playersGamepad.left_bumper && !switchLeftBumper){
            bumperLeft = !bumperLeft;
            switchLeftBumper = true;
            tLeftBumperPressed++;
            iterNumLBumper = iterationCount;
        }
        if(!playersGamepad.left_bumper && switchLeftBumper){
            switchLeftBumper = false;
        }
    }

    @Override
    public void isDpad_Up() {
        if(!playersGamepad.dpad_up && !switchDpad_Up) return;

        if(playersGamepad.dpad_up && !switchDpad_Up){
            dpad_Up = !dpad_Up;
            switchDpad_Up = true;
            tDpadUpPressed++;
            iterNumDpadU = iterationCount;
        }
        if(!playersGamepad.dpad_up && switchDpad_Up){
            switchDpad_Up = false;
        }
    }

    @Override
    public void isDpad_Down() {
        if(!playersGamepad.dpad_down && !switchDpad_Down) return;

        if(playersGamepad.dpad_down && !switchDpad_Down){
            dpad_Down = !dpad_Down;
            switchDpad_Down = true;
            tDpadDownPressed++;
            iterNumDpadD = iterationCount;
        }
        if(!playersGamepad.dpad_down && switchDpad_Down){
            switchDpad_Down = false;
        }
    }

    @Override
    public void isDpad_Left() {
        if(!playersGamepad.dpad_left && !switchDpad_Left) return;

        if(playersGamepad.dpad_left && !switchDpad_Left){
            dpad_Left = !dpad_Left;
            switchDpad_Left = true;
            tDpadLeftPressed++;
            iterNumDpadL = iterationCount;
        }
        if(!playersGamepad.dpad_left && switchDpad_Left){
            switchDpad_Left = false;
        }
    }

    @Override
    public void isDpad_Right() {
        if(!playersGamepad.dpad_right && !switchDpad_Right) return;

        if(playersGamepad.dpad_right && !switchDpad_Right){
            dpad_Right = !dpad_Right;
            switchDpad_Right = true;
            tDpadRightPressed++;
            iterNumDpadR = iterationCount;
        }
        if(!playersGamepad.dpad_right && switchDpad_Right){
            switchDpad_Right = false;
        }
    }

    @Override
    public void isBack_Button() {
        if(!playersGamepad.back && !switchBack) return;

        if(playersGamepad.back && !switchBack){
            buttonBack = !buttonBack;
            switchBack = true;
            tBackPressed++;
            iterNumBack = iterationCount;
        }
        if(!playersGamepad.back && switchBack){
            switchBack = false;
        }
    }

    @Override
    public void isStart_Button() {
        if(!playersGamepad.start && !switchStart) return;

        if(playersGamepad.start && !switchStart){
            buttonStart = !buttonStart;
            switchStart = true;
            tStartPressed++;
            iterNumStart = iterationCount;
        }
        if(!playersGamepad.start && switchStart){
            switchStart = false;
        }
    }

    @Override
    public void isA_and_Left_Trigger() {
        if(!(buttonA && triggerLeft) && !switchA_and_Left_Trigger) return;

        if(buttonA && triggerLeft && !switchA_and_Left_Trigger && iterNumA == iterNumLTrigger){
            buttonA = false;
            triggerLeft = false;

            a_and_Left_Trigger = !a_and_Left_Trigger;
            switchA_and_Left_Trigger = true;
            tA_and_Left_Trigger++;
        }
        if(!a_and_Left_Trigger && switchA_and_Left_Trigger){
            switchA_and_Left_Trigger = false;
        }
    }

    @Override
    public void isTouchpad() {
        if(!playersGamepad.touchpad_finger_1 && !switchTouchpad) return;

        if(playersGamepad.touchpad_finger_1 && !switchTouchpad){
            touchpad = !touchpad;
            switchTouchpad = true;
            tTouchPressed++;
        }
        if(!playersGamepad.touchpad_finger_1 && switchTouchpad){
            switchTouchpad = false;
        }
    }

    public void setIterationCount(int iterationCount) {
        this.iterationCount = iterationCount;
    }


    @Override
    protected void showDataExt() {
        telemetry.addData("left Stick", "X %s Y%s", cosA, sinA);
        telemetry.addData("right Stick", "X %s Y%s", cosB, sinB);
        telemetry.addData("Buttons", "A:%s B:%s X:%s Y:%s", buttonA, buttonB, buttonX, buttonY);
        telemetry.addData("Bumpers", "L:%s R:%s", bumperLeft, bumperRight);
        telemetry.addData("Triggers", "L:%s R:%s", triggerLeft, triggerRight);
        telemetry.addData("DPad", "U:%s D:%s L:%s R:%s", dpad_Up, dpad_Down, dpad_Left, dpad_Right);
        telemetry.addData("Bumper times", "L: %s", tLeftBumperPressed);
    }
}