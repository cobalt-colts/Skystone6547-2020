package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

@TeleOp(name = "Gyro Calibration Test", group = "test")
@Disabled
public class calibrateTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547State bot = new DriveTrain6547State(this);

        telemetry.log().add("ready to start");
        waitForStart();

        telemetry.log().add("push a to calibrate");
        while (opModeIsActive())
        {
            bot.updateGamepads();

            if (bot.a1.onPress())
            {
                bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, bot.getIMUAngle());
            }
            telemetry.addData("IMU angle", bot.getIMUAngle());
            telemetry.addData("road runner gyro angle", bot.getRawExternalHeading());
            telemetry.update();
        }
    }
}
