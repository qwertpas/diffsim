package sim3;

import java.awt.Color;
import sim3.GraphicDebug.Serie;


public class UserCode{

    static double lPower;
    static double rPower;

    public static void initialize(){ //this function is run once when the robot starts
        GraphicDebug.turnOnAll(); //displaying the graphs
    }

    public static void execute(){ //this function is run 50 times a second (every 0.02 second)

        double forward = -Controls.rawY;
        double turn = Controls.rawX;
        
        Main.robot.setDrivePowers(forward+turn, -forward-turn, forward-turn, -forward+turn); //power ranges from -1 to 1

        graph(); //updating the graphs
    }





    // Motion graphs
    static Serie w1s1 = new Serie(Color.BLUE, 3);
    static Serie w1s2 = new Serie(Color.RED, 3);
    static GraphicDebug w1 = new GraphicDebug("forces", new Serie[]{w1s1, w1s2});

    static Serie w2s1 = new Serie(Color.BLUE, 3);
    static GraphicDebug w2 = new GraphicDebug("angvelo", new Serie[]{w2s1});
    
    private static void graph(){
        w1s1.addPoint(Controls.rawX, Controls.rawY);
        // w1s2.addPoint(Main.elaspedTime, Main.robot.leftModule.force.x);

        w2s1.addPoint(Main.elaspedTime, Main.robot.angVelo);

        GraphicDebug.paintAll();
    }




}