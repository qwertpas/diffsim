package sim3;

class Motor{

    double voltage = 0; //ranges from 0-12V
    double angVelo = 0; //angular velocity in radians per second
    double torque = 0; //newton*meters

    void setVoltage(double voltage_input){
        voltage = voltage_input;
    }

    void setAngSpeed(double radPerSec_input){
        angVelo = radPerSec_input;
    }

    double getTorque(){ //input is radians per second
        torque = Constants.STALL_TORQUE.getDouble() * ((voltage/12.0) - (angVelo / Constants.FREE_SPEED.getDouble()));
        System.out.println("torq: " + torque);

        return torque;
    }


}