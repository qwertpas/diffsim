package sim3;

import org.ejml.simple.SimpleMatrix;
import sim3.Util.*;

class DiffModule {

    Motor topMotor;
    Motor bottomMotor;

    Vector2D moduleTranslation = new Vector2D(0, 0); // translational velocity of the module
    double wheelTanVelo;
    double wheelAngVelo; // angular velocity of the wheel
    double moduleAngVelo; // angular velocity of the module
    double moduleAngle = 0;

    double wheelTorque;

    Vector2D force = new Vector2D(0, 0);

    double dt;
    double lastTime;

    DiffModule() {
        topMotor = new Motor();
        bottomMotor = new Motor();
        lastTime = System.nanoTime();
    }

    void update() {
        // wheelTanVelo = (new Vector2D(moduleAngle)).dotProduct(moduleTranslation); // tangential velocity of the wheel
        wheelTanVelo = moduleTranslation.x; // tangential velocity of the wheel

        wheelAngVelo = wheelTanVelo / Constants.WHEEL_RADIUS.getDouble(); // tangential velocity = radius * angular
                                                                          // velocity

        updateMotorSpeeds();
        // updateModuleAngle();

        wheelTorque = (topMotor.getTorque() - bottomMotor.getTorque()) * Constants.GEAR_RATIO.getDouble();

        wheelTorque = Util.applyFrictions(wheelTorque, wheelAngVelo, 1, 1, 0.01);

        double force_mag = wheelTorque / Constants.WHEEL_RADIUS.getDouble(); // F=ma
        force = new Vector2D(force_mag, 0);
    }

    void setTranslation(Vector2D moduleTranslation_input) {
        this.moduleTranslation = moduleTranslation_input;
    }

    void updateModuleAngle() {
        dt = (System.nanoTime() - lastTime) / 1e+9; // change in time (seconds) used for integrating
        lastTime = System.nanoTime();

        double moduleTorque = topMotor.getTorque() + bottomMotor.getTorque();
        double moduleAngAccel = moduleTorque / Constants.MODULE_ROT_INERTIA.getDouble();
        moduleAngVelo = moduleAngAccel * dt + moduleAngVelo; // integration
        moduleAngle = moduleAngVelo * dt + moduleAngle; // second integration
    }

    void updateMotorSpeeds() {
        SimpleMatrix wheelMatrix = new SimpleMatrix(new double[][] { { wheelAngVelo }, { moduleAngVelo } });
        double g = Constants.GEAR_RATIO.getDouble();

        SimpleMatrix diffMatrix = new SimpleMatrix(new double[][] { { 1 / g, -1 / g }, { 0.5, 0.5 } });

        SimpleMatrix ringsMatrix = diffMatrix.solve(wheelMatrix);

        topMotor.setAngSpeed(ringsMatrix.get(0, 0));
        bottomMotor.setAngSpeed(ringsMatrix.get(1, 0));
    }


} 