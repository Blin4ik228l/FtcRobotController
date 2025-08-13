package org.firstinspires.ftc.teamcode.Modules.Players;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Module;

public abstract class Player extends Module {
    public Gamepad playersGamepad;

    public Player(Telemetry telemetry) {
        super(telemetry);
    }

    public void play(){};//Основной метод, где программа будет выполняться

    public void showData(){};

    public class JoystickActivity implements JoystickButtons {
        public boolean buttonX = false, buttonA = false, buttonB = false, buttonY = false,
                buttonBack = false, buttonStart = false,
                bumperRight = false, bumperLeft = false,
                triggerRight = false, triggerLeft = false,
                dpad_Up = false, dpad_Down = false, dpad_Left = false, dpad_Right = false;

        public boolean switchA = false, switchX = false, switchB = false, switchY = false, switchBack = false, switchStart = false,
                switchRightBumper = false, switchLeftBumper = false,
                switchRightTrigger = false, switchLeftTrigger = false,
                switchDpad_Up = false, switchDpad_Down = false, switchDpad_Left = false, switchDpad_Right = false;
        public int tAPressed = 0, tBPressed = 0, tXPressed = 0, tYPressed = 0,
                tRightTriggerPressed = 0, tLeftTriggerPressed = 0, tRightBumperPressed = 0, tLeftBumperPressed = 0,
                tDpadDownPressed = 0, tDpadUpPressed = 0, tDpadLeftPressed = 0, tDpadRightPressed = 0,
                tBackPressed, tStartPressed = 0;

        public void checkActivity(){
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
        }

        @Override
        public void isA_Button() {
            if(!playersGamepad.a && !switchA) return;

            if(playersGamepad.a && !switchA){
                buttonA = !buttonA;
                switchA = true;
                tAPressed++;
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
            }
            if(!playersGamepad.start && switchStart){
                switchStart = false;
            }
        }
    }
}
