package org.firstinspires.ftc.teamcode.util.pathfinding;

import org.firstinspires.ftc.teamcode.util.pathfinding.aStar.ExampleFactory;
import org.firstinspires.ftc.teamcode.util.pathfinding.aStar.ExampleNode;
import org.firstinspires.ftc.teamcode.util.pathfinding.aStar.Map;

public class SkyStoneField {
	
	Map<ExampleNode> myMap;
	Rectangle blueLoadingZone;
	Rectangle bump;
	Rectangle blueBridge;
	Rectangle blueSide;
	double robotSize = 18;
	
	public SkyStoneField()
	{
		myMap = new Map<ExampleNode>(144, 144, new ExampleFactory());
		blueLoadingZone = new Rectangle(0, 0, 24,24);
		bump = new Rectangle(64, 48, 80, 96);
		blueBridge = new Rectangle(68, 96, 76, 144);
		blueSide = new Rectangle(72, 72, 144, 144);
		myMap.fillRectangle(blueLoadingZone);
		myMap.fillRectangle(bump);
		myMap.fillRectangle(blueBridge);
		myMap.fillRectangle(blueSide);
	}
	public Map<ExampleNode> getMap() {
		return myMap;
	}

}
