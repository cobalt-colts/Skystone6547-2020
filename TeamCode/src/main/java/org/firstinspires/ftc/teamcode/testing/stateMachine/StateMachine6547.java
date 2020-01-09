package org.firstinspires.ftc.teamcode.testing.stateMachine;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.SkyStone6547Qualifter;

import java.util.ArrayList;
import java.util.List;

@Disabled
public class StateMachine6547 {

    int state = 0;

    SkyStone6547Qualifter bot;
    List<StateManager> states = new ArrayList<>();
    public StateMachine6547(SkyStone6547Qualifter bot)
    {
        this.bot = bot;
    }
    public void addState(StateManager stateManager)
    {
        states.add(stateManager);
    }

    public void doState()
    {
        states.get(state).runStates();
        if (states.get(state).isDone())
        {
            state++;
        }
    }

    public void setState(int state) {
        this.state = state;
    }

    public int getState() {
        return state;
    }
}
