package sim3;

import org.ejml.simple.SimpleMatrix;

class DiffModule {

    Motor topMotor;
    Motor bottomMotor;

    double radPerSec;
    double torque;
    double force;

    DiffModule() {
        topMotor = new Motor();
        bottomMotor = new Motor();
    }

    double getForce() {
        torque = (topMotor.getTorque() - bottomMotor.getTorque()) * Constants.GEAR_RATIO.getDouble();
        torque = Util.applyFrictions(torque, radPerSec, 0, 0, 0);

        force = torque / Constants.WHEEL_RADIUS.getDouble();
        return force;
    }

    void distributeVelocities(double wheelVelocity, double wheelOmega) {
        // double g = Constants.GEAR_RATIO.getDouble();
        double g = 1;

        SimpleMatrix wheelMatrix = new SimpleMatrix(new double[][] { { wheelVelocity },
                                                                     { wheelOmega    } });
        SimpleMatrix diffMatrix = new SimpleMatrix(new double[][] { { 1/g , -1/g },
                                                                    { 0.5 , 0.5  } });

        SimpleMatrix ringsMatrix = diffMatrix.solve(wheelMatrix);
        System.out.println(ringsMatrix.get(0, 0));
        System.out.println(ringsMatrix.get(1, 0));
    }

}