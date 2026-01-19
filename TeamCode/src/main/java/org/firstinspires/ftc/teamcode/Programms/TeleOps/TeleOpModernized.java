package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl1.PlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutoPlayerClass;
import org.firstinspires.ftc.teamcode.FileSystem;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.TelemetrySettings;

public abstract class TeleOpModernized extends OpMode {
    public RobotClass robot;
    public PlayerClass1 player1;
    public AutoPlayerClass autoPlayerClass;
    public JoystickActivityClass joystickActivityClass;
    public JoystickActivityClass joystickActivityClass2;
    public TelemetrySettings telemetrySettings;
    public ElapsedTime updateTime;
    public FileSystem fileSystem;
    public int iterationCount = 1;

    public void initAfterRobot(){
        joystickActivityClass = new JoystickActivityClass(gamepad1, this);
        joystickActivityClass2 = new JoystickActivityClass(gamepad2, this);

        player1 = new PlayerClass1(joystickActivityClass, robot.drivetrain, robot.innerRunTime,this);
        autoPlayerClass = new AutoPlayerClass(joystickActivityClass, robot.collector, robot.innerRunTime,this);

        fileSystem = new FileSystem(autoPlayerClass, robot,this);

        telemetrySettings = new TelemetrySettings(this, null, this);

        updateTime = new ElapsedTime();
    }
    @Override
    public void init_loop() {
        setAll();
        updateAll();
        showAll();
    }

    @Override
    public void start() {
        robot.resetTimer();
    }

    @Override
    public void loop() {
        setAll();
        updateAll();
        executeAll();
        showAll();

        iterationCount++;
        updateTime.reset();
    }

    @Override
    public void stop() {
        fileSystem.stop();
    }

    public void extUpdate(){

    }
    public void extExecute(){

    }
    public void extShow(){

    }
    private void setAll(){
        joystickActivityClass.setIterationCount(iterationCount);
        joystickActivityClass2.setIterationCount(iterationCount);
        robot.setIterationCount(iterationCount);
    }
    private void updateAll() {
        joystickActivityClass.update();
        joystickActivityClass2.update();

        telemetrySettings.update();

        robot.update();

        autoPlayerClass.setFields(robot.drivetrain.positionRobotController);

        fileSystem.update();

        extUpdate();
    }
    private void executeAll(){
        player1.execute();
        autoPlayerClass.execute();

        extExecute();
    }
    private void showAll(){
        telemetrySettings.showData();
        telemetry.addData("Update time / Frequency", "%.2f sec %.2f Hz", updateTime.seconds(), 1 / updateTime.seconds());
    }
}

