package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Examples.Players.PL0.MainSystem;
import org.firstinspires.ftc.teamcode.Modules.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl1.SemiAutoPlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutoPlayerClass2;
import org.firstinspires.ftc.teamcode.FileSystem;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.TelemetrySettings;

public abstract class TeleOpModernized extends OpMode {
    public RobotClass robot;
    public SemiAutoPlayerClass1 semiAutoPlayerClass1;
    public AutoPlayerClass2 autoPlayerClass2;
    public MainSystem mainSystem;
    public JoystickActivityClass joystickActivityClass;
    public JoystickActivityClass joystickActivityClass2;
    public TelemetrySettings telemetrySettings;
    public FileSystem fileSystem;
    public int iterationCount = 1;
    public ElapsedTime updateTime;

    public void initAfterRobot(){
        joystickActivityClass = new JoystickActivityClass(gamepad1, this);
        joystickActivityClass2 = new JoystickActivityClass(gamepad2, this);

        semiAutoPlayerClass1 = new SemiAutoPlayerClass1(joystickActivityClass, robot.drivetrain, robot.innerRunTime,this);
        autoPlayerClass2 = new AutoPlayerClass2(joystickActivityClass, robot.collector, robot.innerRunTime,this);

        mainSystem = new MainSystem(semiAutoPlayerClass1, autoPlayerClass2, this);

        fileSystem = new FileSystem(autoPlayerClass2, robot,this);

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
        robot.setIteration(iterationCount);
    }
    private void updateAll() {
        joystickActivityClass.update();
        joystickActivityClass2.update();

        telemetrySettings.update();

        robot.update();

        autoPlayerClass2.setFields(robot.drivetrain.positionRobotController);

//        fileSystem.update();

        extUpdate();
    }
    private void executeAll(){
        mainSystem.execute();

        semiAutoPlayerClass1.execute();
        autoPlayerClass2.execute();

        extExecute();
    }
    private void showAll(){
        telemetrySettings.showData();
        telemetry.addData("Update time / Frequency", "%.2f sec %.2f Hz", updateTime.seconds(), 1 / updateTime.seconds());
    }
}

