package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "odo my odo")
public class TestZaOdometryWorldo extends LinearOpMode {

    DcMotor odo;
    @Override
    public void runOpMode() throws InterruptedException {
        odo = hardwareMap.get(DcMotor.class, "odo");
        odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odo.setDirection(DcMotorSimple.Direction.REVERSE);
        odo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.log().add("ready to start");
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("Odometry current pos", odo.getCurrentPosition());
            telemetry.update();
        }
    }
}
