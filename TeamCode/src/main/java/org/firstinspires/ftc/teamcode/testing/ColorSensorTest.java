/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.DriveTrain6547State;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */

@TeleOp
@Config
public class ColorSensorTest extends LinearOpMode {
    public static double INTAKE_SPEED = .5;
    public static double INTAKE_SPEED_2 = .25;
    @Override
    public void runOpMode() {



        // get a reference to the color sensor.
        DriveTrain6547State bot = new DriveTrain6547State(this);

        // wait for the start button to be pressed.
        waitForStart();

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)

//            boolean isStoneAtIntake = bot.isStone(bot.intakeColorSensor);
//            boolean isStoneAtEnd = isStoneAtEnd(bot);
//
//            if (isStoneAtIntake)
//            {
//                //If a stone is under the bot, go slower so it reaches the end
//                bot.intake(INTAKE_SPEED_2);
//            }
//            else if (isStoneAtEnd)
//            {
//                //stone at end.
//                bot.stopIntake();
//            }
//            else //if nothing is detected, intake at regular power
//            {
//                bot.intake(INTAKE_SPEED);
//            }
            bot.runIntakeUntilStone(INTAKE_SPEED);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Is Intake Stone: " , bot.isStoneAtIntake());
            telemetry.addData("Intake Alpha", bot.intakeColorSensor.alpha());
            telemetry.addData("Intake Red  ", bot.intakeColorSensor.red());
            telemetry.addData("Intake Green", bot.intakeColorSensor.green());
            telemetry.addData("Intake Blue ", bot.intakeColorSensor.blue());
            telemetry.addData("Is End Stone", bot.isStone(bot.endColorSensor));
            telemetry.addData("End Alpha", bot.endColorSensor.alpha());
            telemetry.addData("End Red  ", bot.endColorSensor.red());
            telemetry.addData("End Green", bot.endColorSensor.green());
            telemetry.addData("End Blue ", bot.endColorSensor.blue());
            telemetry.addData("Is Right End SkyStone", bot.isStone(bot.rightEndColorSensor));
            telemetry.addData("Right End Alpha", bot.rightEndColorSensor.alpha());
            telemetry.addData("Right End Red  ", bot.rightEndColorSensor.red());
            telemetry.addData("Right End Green", bot.rightEndColorSensor.green());
            telemetry.addData("Right End Blue ", bot.rightEndColorSensor.blue());

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.

            telemetry.update();
        }
    }
    public boolean isStoneAtEnd(DriveTrain6547State bot)
    {
        if (bot.isStone(bot.rightEndColorSensor) || bot.isStone(bot.endColorSensor))
        {
            return true;
        }
        return false;
    }
    public boolean isStone(ColorSensor colorSensor)
    {
        return colorSensor.red()>100;
    }
}
