package org.firstinspires.ftc.teamcode.util.pathfindingOLD;

public class Rectangle {

    private int x1;
    private int x2;
    private int y1;
    private int y2;
    private int oldx1, oldx2, oldy1, oldy2;
    private double scale = 1;
    int[][] points;


    public Rectangle(int x1, int y1, int x2, int y2) {
        this.x1 = x1;
        this.oldx1 = x1;
        this.x2 = x2;
        this.oldx2 = x2;
        this.y1 = y1;
        this.oldy1 = y1;
        this.y2 = y2;
        this.oldy2 = y2;
    }

    public void setScale(double scale) {
        this.scale = scale;
        x1 = (int) (oldx1 * scale);
        x2 = (int) (oldx2 * scale);
        y1 = (int) (oldy1 * scale);
        y2 = (int) (oldy2 * scale);
        fillRect();
    }

    public int[][] fillRect() {
        int xRange = Math.abs(x2 - x1);
        int yRange = Math.abs(y2 - y1);
        int totalRange = xRange * yRange;
        int[][][] rect = new int[xRange][yRange][2];
        for (int i = 0; i < xRange; i++) {
            for (int j = 0; j < yRange; j++) {
                rect[i][j] = new int[]{i + x1, j + y1};
            }
        }
        int[][] newPoints = new int[totalRange][2];
        int k = 0;
        for (int i = 0; i < xRange; i++) {
            k++;
            for (int j = 0; j < yRange; j++) {
                newPoints[k][0] = rect[i][j][0];
                newPoints[k][1] = rect[i][j][1];
            }
        }
        return points;
    }

    public int[][] getPoints() {
        return points;
    }
}
