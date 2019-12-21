package org.firstinspires.ftc.teamcode.util.pathfinding;

import java.util.List;

import org.firstinspires.ftc.teamcode.util.pathfinding.aStar.ExampleNode;

public class TestPath {
	
	public static void main(String[] args) {
		SkyStoneField field = new SkyStoneField();
		field.getMap().setScale(.25);
		
		List<ExampleNode> path = field.getMap().findPath(30, 0, 120, 48);

        for (int i = 0; i < path.size(); i++) {
            System.out.print("(" + path.get(i).getxPosition() + ", " + path.get(i).getyPosition() + ") -> ");
        }
		field.getMap().printPath(path);
		
	}

}
