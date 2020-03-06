package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

@TeleOp
@Disabled
public class limitSwitchTest extends LinearOpMode {

    AnalogInput limitSwitch3;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547State bot = new DriveTrain6547State(this);
        bot.limitSwitch = hardwareMap.get(AnalogInput.class, "limitSwitch");

        AnalogInput limitSwitch2 = hardwareMap.get(AnalogInput.class, "limitSwitch");
        limitSwitch3 = hardwareMap.get(AnalogInput.class, "limitSwitch");

        telemetry.addData("ready to start","");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Limit Switch Voltage", bot.limitSwitch.getVoltage());
            telemetry.addData("Limit Switch Voltage 2",limitSwitch2.getVoltage());
            telemetry.addData("Limit Switch Voltage 3",limitSwitch3.getVoltage());
            telemetry.update();
        }
    }
}
