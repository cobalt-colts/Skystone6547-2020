package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

@TeleOp
public class limitSwitchTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547State bot = new DriveTrain6547State(this);

        AnalogInput switch2 = hardwareMap.get(AnalogInput.class, "ls2");

        telemetry.addData("ready to start","");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Limit Switch Voltage", bot.limitSwitch.getVoltage());
            telemetry.addData("Limit Switch Port 0 Voltage",switch2.getVoltage());
            telemetry.update();
        }
    }
}
