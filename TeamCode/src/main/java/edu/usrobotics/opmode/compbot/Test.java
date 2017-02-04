package edu.usrobotics.opmode.compbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by dsiegler19 on 1/27/17.
 */

@Autonomous(name="Compbot Auto Test", group="Compbot")
public class Test extends OpMode{

    public ColorSensor buttonPresserColorSensorRight;

    @Override
    public void init(){

        buttonPresserColorSensorRight = hardwareMap.colorSensor.get("cs");
        buttonPresserColorSensorRight.setI2cAddress(I2cAddr.create7bit(0x2e));
        buttonPresserColorSensorRight.enableLed(true);

    }

    @Override
    public void loop() {

        telemetry.addData("color sensor", buttonPresserColorSensorRight.red() + " " + buttonPresserColorSensorRight.green() + " " + buttonPresserColorSensorRight.blue());

        telemetry.update();

    }

}
