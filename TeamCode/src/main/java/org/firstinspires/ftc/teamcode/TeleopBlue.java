package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TeleopBlue extends OpMode {
    InitClass initClass;
    ElapsedTime runtime = new ElapsedTime();
    DcMotorEx rightB = null;
double last_moment_serv = 0.0, last_moment_switch = 0.0, last_moment_free = 0.0,
        moment_diff_serv = 0.0, moment_diff_switch = 0.0, moment_diff_free = 0.0;

Servo sHook;
    @Override
    public void init() {
        sHook = hardwareMap.get(Servo.class, "sHook");
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        moment_diff_serv = runtime.milliseconds() - last_moment_serv;
        moment_diff_switch = runtime.milliseconds() - last_moment_switch;

        if (gamepad1.a && moment_diff_serv > 200) {
            if (sHook.getPosition() == CONSTS.CLOSE) {
                sHook.setPosition(CONSTS.OPEN);
                last_moment_serv = runtime.milliseconds();
            } else {
                sHook.setPosition(CONSTS.CLOSE);
                last_moment_serv = runtime.milliseconds();
            }
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
