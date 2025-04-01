package org.firstinspires.ftc.teamcode.Game.Players.Joystick;

public interface Joystick {
    boolean isA_Button();
    boolean isB_Button();
    boolean isX_Button();
    boolean isY_Button();

    boolean isRight_Trigger();
    boolean isLeft_Trigger();

    boolean isRight_Bumper();
    boolean isLeft_Bumper();

    boolean isDpad_Up();
    boolean isDpad_Down();
    boolean isDpad_Left();
    boolean isDpad_Right();

    boolean isBack_Button();
    boolean isStart_Button();
}
