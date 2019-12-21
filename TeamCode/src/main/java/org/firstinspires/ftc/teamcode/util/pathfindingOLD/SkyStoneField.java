package org.firstinspires.ftc.teamcode.util.pathfindingOLD;

import org.firstinspires.ftc.teamcode.util.pathfindingOLD.aStar.AStar;
import org.firstinspires.ftc.teamcode.util.pathfindingOLD.aStar.Node;

import java.util.ArrayList;
import java.util.List;

public class SkyStoneField {

    final int size = 144;
    int rows = size;
    int cols = size;
    int scale = 1;

    Node initPos;
    Node targetPos;

    AStar aStar;

    Rectangle bridgeWalls;
    Rectangle redHumanPlayer;
    Rectangle blueHumanPlayer;
    Rectangle redArea;
    Rectangle blueArea;

    List<Rectangle> blocks = new ArrayList<>();

    public SkyStoneField(Node initPos, Node targetPos, int scale)
    {
        this.scale = scale;
        blocks.add(bridgeWalls);
        blocks.add(redArea);
        blocks.add(redHumanPlayer);
        blocks.add(blueHumanPlayer);
        blocks.add(blueArea);
        aStar = new AStar(rows,cols, initPos,targetPos);
    }
    public SkyStoneField(Node initPos, Node targetPos) {
        this(initPos, targetPos,1);
    }
    public void setScale(int scale)
    {
        for (int i=0; i < blocks.size(); i++)
        {
            blocks.get(i).setScale(scale);
        }
    }

}
