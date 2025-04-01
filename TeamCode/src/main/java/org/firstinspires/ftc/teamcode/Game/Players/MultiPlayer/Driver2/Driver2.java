package org.firstinspires.ftc.teamcode.Game.Players.MultiPlayer.Driver2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Game.Players.Player;
import org.firstinspires.ftc.teamcode.Utils.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.Game.Robot.Modules.Usable.ServosService;
import org.firstinspires.ftc.teamcode.Game.Robot.Modules.Usable.TeleSkope;

public class Driver2 implements Player, ConstsTeleskope {
    /*
    Второй игрок - второй оператор.
    Отвечает за захват и телескоп.
     */
    public Driver2(OpMode op){
        this.op = op;

        servosService = new ServosService(op);
        teleSkope = new TeleSkope(op, servosService);

        joystick2 = new Joystick2(op.gamepad2);
        g2 = joystick2.gamepad;
    }
    public OpMode op;
    public Joystick2 joystick2;
    public Gamepad g2;

    public ServosService servosService;
    public TeleSkope teleSkope;

    public double horizontalPos = CLOSE_POS_HORIZONTAL;

    @Override
    public void init() {
        servosService.init();
        teleSkope.init();
    }

    @Override
    public void functional() {
        double leftStickY = -g2.left_stick_y;

        double upStandingVel = -g2.right_stick_y;

        teleSkope.setTeleskope(upStandingVel, horizontalPos);

        switch (joystick2.dPadUpDownAction()){
            case 0:
                horizontalPos = CLOSE_POS_HORIZONTAL2;
                break;

            case 1:
                horizontalPos = CLOSE_POS_HORIZONTAL2 + 0.05;
                break;

            case 2:
                horizontalPos = CLOSE_POS_HORIZONTAL2 + 0.1;
                break;

            case 3:
                horizontalPos = CLOSE_POS_HORIZONTAL2 + 0.15;
                break;

            case 4:
                horizontalPos = CLOSE_POS_HORIZONTAL2 + 0.2;
                break;

            case 5:
                horizontalPos = OPEN_POS_HORIZONTAL2;
                break;
        }

        switch (joystick2.triggerLeftRightAction()) {
            case 0:
                teleSkope.setTeleskopeHeight(0, joystick2);
                break;

            case 1:
                teleSkope.setTeleskopeHeight(17, joystick2);
                break;

            case 2:
                teleSkope.setTeleskopeHeight(55, joystick2);
                break;

//            case 3:
//                teleSkope.setTeleskopeHeight(70, joysticks);
//                break;
        }

        if(joystick2.isY_Button())
        {
            teleSkope.setFlip(ConstsTeleskope.HANG_POS_FLIP);
            joystick2.isB_Button = false;
        }
        if (joystick2.isB_Button() )
        {
            teleSkope.setFlip(ConstsTeleskope.TAKE_POS_FLIP);
            joystick2.isY_Button = false;
        }
        if (!joystick2.isB_Button() && joystick2.isY_Button())
        {
            teleSkope.setFlip(ConstsTeleskope.MIDLE_POS_FLIP);
        }


        if (joystick2.isA_Button()){
            if(servosService.getHook().getPosition() == OPEN_POS_HOOK)
            {
                servosService.getHook().setPosition(CLOSE_POS_HOOK);
            }else
            {
                teleSkope.setHook(ConstsTeleskope.OPEN_POS_HOOK);
            }
            joystick2.isA_Button = false;
        }

        if(g2.x) {
            while (g2.x) {
                teleSkope.setVelUpStandingTeleOp(-0.7);
            }
            servosService.getHook().setPosition(OPEN_POS_HOOK);
            teleSkope.setTeleskopeHeight(17, joystick2);
            joystick2.tRightTriggerPressed = 1;
        }
    }

    @Override
    public void interruptJoystick() {
        joystick2.interrupt();
    }
}
