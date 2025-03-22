package org.firstinspires.ftc.teamcode.OpModes.Players;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Groups.Joysticks;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Groups.ServosService;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Groups.TeleSkope;

public class Player2 extends Thread implements Module, ConstsTeleskope {
    public Gamepad g2;

    public double horizontalPos = CLOSE_POS_HORIZONTAL2;

    public final TeleSkope teleSkope;
    public final ServosService servosService;
    public final Joysticks joysticks;

    public Player2(OpMode op){
        joysticks = new Joysticks(op, 2);
        servosService = new ServosService(op);
        teleSkope = new TeleSkope(op, servosService);
    }

    @Override
    public void init() {
        joysticks.init();

        servosService.init();
        teleSkope.init();

    }
    public void startTeleop(){
        this.setDaemon(true);
        this.start();
    }

    @Override
    public void run() {
        while (!this.isInterrupted()){
            teleopPl2();
        }
    }

    // Gamepad 2
    public synchronized void teleopPl2() {
        g2 = joysticks.getGamepad2();

        double leftStickY = -g2.left_stick_y;

        double upStandingVel = -g2.right_stick_y;

        switch (joysticks.getDpadUp(g2.dpad_down)){
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

        switch (joysticks.getGearTele()) {
            case 0:
                teleSkope.setTeleskopeHeight(0, joysticks);
                break;

            case 1:
                teleSkope.setTeleskopeHeight(17, joysticks);
                break;

            case 2:
                teleSkope.setTeleskopeHeight(45, joysticks);
                break;

//            case 3:
//                teleSkope.setTeleskopeHeight(70, joysticks);
//                break;
        }

        teleSkope.setTeleskopeTele(upStandingVel, horizontalPos, joysticks);

        if(joysticks.isY_G2())
        {
            teleSkope.setFlip(ConstsTeleskope.HANG_POS_FLIP);
            joysticks.isB_g2 = false;
        }
        if (joysticks.isB_G2() )
        {
            teleSkope.setFlip(ConstsTeleskope.TAKE_POS_FLIP);
            joysticks.isY_g2 = false;
        }
        if (!joysticks.isB_G2() && !joysticks.isY_G2())
        {
            teleSkope.setFlip(ConstsTeleskope.MIDLE_POS_FLIP);
        }


        if (joysticks.isA_G2()){
            if(servosService.getHook().getPosition() == OPEN_POS_HOOK)
            {
                servosService.getHook().setPosition(CLOSE_POS_HOOK);
            }else
            {
                teleSkope.setHook(ConstsTeleskope.OPEN_POS_HOOK);
            }
            joysticks.isA_g2 = false;
        }

        if(joysticks.getGamepad2().x) {
            while (joysticks.getGamepad2().x) {
                teleSkope.setVelUpStandingTeleOp(-0.7);
            }
            servosService.getHook().setPosition(OPEN_POS_HOOK);
            teleSkope.setTeleskopeHeight(17, joysticks);
            joysticks.gearTele = 1;
        }

    }
}
