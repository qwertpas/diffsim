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
        
        Main.robot.setDrivePowers(forward, -forward, forward, -forward); //power ranges from -1 to 1

        graph(); //updating the graphs
    }





    // Motion graphs
    static Serie currentPositionSerie = new Serie(Color.BLUE, 3);
    static GraphicDebug positionWindow = new GraphicDebug("Position", new Serie[]{currentPositionSerie});

    static Serie currentVelocitySerie = new Serie(Color.BLUE, 3);
    static GraphicDebug velocityWindow = new GraphicDebug("Velocity", new Serie[]{currentVelocitySerie});
    
    private static void graph(){
        currentVelocitySerie.addPoint(Main.elaspedTime, Util.metersToFeet(Main.robot.linVelo.getMagnitude()));

        GraphicDebug.paintAll();
    }




}