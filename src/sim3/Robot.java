package sim3;

import sim3.Util.*;

public class Robot{

    DiffModule leftModule, rightModule;

    Vector2D position = new Vector2D(10, 10);
    double heading = 0;

    Vector2D linVelo = new Vector2D(0, 0);
    double angVelo = 0;

    Vector2D linAccel = new Vector2D(0, 0);
    double angAccel = 0;

    double torqueMotors = 0;
    double torqueNet = 0;
    Vector2D forceNet = new Vector2D(0, 0);
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

        leftModule.update();
        rightModule.update();

        forceNet = leftModule.force.add(rightModule.force).rotate(heading); //force on robot center of mass
        torqueNet = calcRobotTorque(leftModule.force, rightModule.force); //torque around robot center

        linAccel = forceNet.scalarDiv(Constants.ROBOT_MASS.getDouble()); //linear acceleration of robot center of mass
        angAccel = torqueNet / Constants.ROBOT_ROT_INERTIA; //angular acceleration around robot center
        
        linVelo = linAccel.scalarMult(dt).add(linVelo); //linear velocity of robot center of mass
        angVelo = angVelo + angAccel * dt; //angular velocity around robot center

        leftModule.setTranslation(linVelo.scalarAdd(angVelo * Constants.HALF_DIST_BETWEEN_WHEELS));
        rightModule.setTranslation(linVelo.scalarAdd(-angVelo * Constants.HALF_DIST_BETWEEN_WHEELS));
        // leftModule.setTranslation(linVelo);
        // rightModule.setTranslation(linVelo);

        heading = heading + angVelo * dt; //integrating angVelo

        position = linVelo.scalarMult(dt).add(position);
    }

    private double calcRobotTorque(Vector2D forceL, Vector2D forceR){
        torqueMotors = (forceR.x - forceL.x) * Constants.HALF_DIST_BETWEEN_WHEELS; //torque around center of robot
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