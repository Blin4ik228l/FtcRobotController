package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config;

public class ExpansionHubDevices extends DevicesConfig{
    public ExpansionHubDevices(){
        /**Важное примечение если между портами ничего нет например 0(atacched), 1(detached), 2(attached) - то поставитье "" вместо пропуска
         * Пример(для любого устрйства)
         * setMotorNames(servoName1, "", servoName3)
         **/
        new Builder()
                .setMotorNames("rightBack", "rightFront", "leftFront", "leftBack")
                .setServoNames("")
                .setI2CDeviceNames("")
                .setDigDeviceNames("")
                .setAnalogDeviceNames("");
    }
}
