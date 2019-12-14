package sim3;

import sim3.Util.*;

public class Robot{

    DiffModule leftModule, rightModule;

    Vector2D position = new Vector2D(5, 5);
    double heading = 0;

    Vector2D linVelo = new Vector2D(0, 0);
    double angVelo = 0;

    Vector2D linAccel = new Vector2D(0, 0);
    double angAccel = 0;

    double veloL = 0;
    double veloR = 0;

    double distL = 0;
    double distR = 0;

    double torqueL;
    double torqueR;
    double torqueNet;
    Vector2D forceNet;
    Boolean slipping = false;

    double dt;
    double lastTime;

    

    Robot(){
        lastTime = System.nanoTime();
        leftModule = new DiffModule();
        rightModule = new DiffModule();
    }

    void update(){
        dt = (System.nanoTime() - lastTime) / 1e+9; //change in time (seconds) used for integrating
        lastTime = System.nanoTime();

        forceNet = leftModule.getForce().add(rightModule.getForce()); //force on robot center of mass
        torqueNet = calcRobotTorque(leftModule.getForce(), rightModule.getForce()); //torque around robot center

        linAccel = forceNet.scalarDiv(Constants.ROBOT_MASS.getDouble()); //linear acceleration of robot center of mass
        angAccel = torqueNet / Constants.ROBOT_ROT_INERTIA; //angular acceleration around robot center
        
        linVelo = linAccel.scalarMult(dt).add(linVelo); //linear velocity of robot center of mass
        angVelo = angVelo + angAccel * dt; //angular velocity around robot center

        leftModule.setTranslation(linVelo.scalarAdd(angVelo * Constants.HALF_DIST_BETWEEN_WHEELS));
        rightModule.setTranslation(linVelo.scalarAdd(-angVelo * Constants.HALF_DIST_BETWEEN_WHEELS));

        heading = heading + angVelo * dt; //integrating angVelo

        position = linVelo.scalarMult(dt).add(position);
    }

    private double calcRobotTorque(Vector2D forceL, Vector2D forceR){
        double torqueMotors = (forceL.getMagnitude() - forceR.getMagnitude()) * Constants.HALF_DIST_BETWEEN_WHEELS; //torque around center of robot
        torqueNet = Util.applyFrictions(torqueMotors, angVelo, Constants.WHEEL_SCRUB_STATIC, Constants.WHEEL_SCRUB_KINE, Constants.WHEEL_SCRUB_FRIC_THRESHOLD.getDouble());
        return torqueNet;
    }

    public void setDrivePowers(double lt, double lb, double rt, double rb){
        leftModule.topMotor.setVoltage(lt*12);
        leftModule.bottomMotor.setVoltage(lb*12);
        rightModule.topMotor.setVoltage(rt*12);
        rightModule.bottomMotor.setVoltage(rb*12);
    }

    


}