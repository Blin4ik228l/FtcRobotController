package org.firstinspires.ftc.teamcode.Programms.Auto.Other;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test", group = "def")
public class Test extends OpMode {
    Servo p1, p2, p3;
    int iterations;
    @Override
    public void init() {
        p1 = hardwareMap.get(Servo.class, "p1");
        p2 = hardwareMap.get(Servo.class, "p2");
        p3 = hardwareMap.get(Servo.class, "p3");
    }

    @Override
    public void loop() {
        if(iterations % 1000 == 0){
            if(p1.getPosition() != 0.0){
                p1.setPosition(0.0);
                p2.setPosition(0.0);
                p3.setPosition(0.0);
            }else {
                p1.setPosition(1.0);
                p2.setPosition(1.0);
                p3.setPosition(1.0);
            }

        }
        iterations++;
    }
}
