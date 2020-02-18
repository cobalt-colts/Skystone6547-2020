package org.firstinspires.ftc.teamcode.testing.stateMachine;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInQualifier.SkyStone6547Qualifter;

import java.util.ArrayList;
import java.util.List;

@Disabled
public class StateTesst6547 extends LinearOpMode {

    SkyStone6547Qualifter bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new SkyStone6547Qualifter(this);

        StateMachine6547 stateMachine = new StateMachine6547(bot);

        List<Action> state0 = new ArrayList<>();
        state0.add(new Action(bot, bot.driveTrainMotors, .5, 300));
        stateMachine.addState(new StateManager(bot,state0));
    }
}
