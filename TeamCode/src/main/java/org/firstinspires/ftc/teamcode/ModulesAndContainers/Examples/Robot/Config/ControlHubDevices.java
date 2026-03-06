package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config;

public class ControlHubDevices extends DevicesConfig{
    public final String gyroskope = "imu";
    public final String webcam1 = "Webcam 1";
    public ControlHubDevices(){
        /**Важное примечение если между портами ничего нет например 0(atacched), 1(detached), 2(attached) - то поставитье "" вместо пропуска
         * Пример(для любого устрйства)
         * setMotorNames(servoName1, "", servoName3)
         **/
        new Builder()
                .setMotorNames("turret", "left", "right")
                .setServoNames("")
                .setI2CDeviceNames("color4", "color5")
                .setDigDeviceNames("")
                .setAnalogDeviceNames("");
    }
}
