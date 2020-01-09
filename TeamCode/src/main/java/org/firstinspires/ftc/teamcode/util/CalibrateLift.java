package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;

/*
This class calibrates the linear slide.  The driver can move the linear slide up and down
and use the A button to set the linear slide's current position as zero
Note: zero is intended to be the lowest the linear slide can go.
 */
@TeleOp(name = "CalibrateLift")
@Disabled
public class CalibrateLift extends SkyStone6547Meet2 {

    public void runOpMode()
    {
        INIT(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            lift.setPower(gamepad1.left_stick_y);
            if (gamepad1.a)
            {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            telemetry.addData("lift current pos", lift.getCurrentPosition());
            telemetry.addData("lift power", lift.getPower());
            telemetry.update();
        }
    }

}