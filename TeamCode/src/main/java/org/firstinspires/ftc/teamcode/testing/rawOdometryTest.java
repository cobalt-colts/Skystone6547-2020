package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Raw Odometry Test", group = "test")
public class rawOdometryTest extends LinearOpMode {

    private DcMotor sideEncoder,frontEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        sideEncoder = hardwareMap.dcMotor.get("sideEncoder");
        frontEncoder = hardwareMap.dcMotor.get("rightFront");

        telemetry.log().add("ready to start");
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("side Encoder Pos", sideEncoder.getCurrentPosition());
            telemetry.addData("perpendicular Encoder Pos",frontEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
