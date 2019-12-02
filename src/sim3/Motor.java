package sim3;

class Motor{

    double voltage = 0; //ranges from 0-12V
    double radPerSec = 0; //angular velocity in radians per second
    double torque = 0; //newton*meters

    void setVoltage(double voltage_input){
        voltage = voltage_input;
    }

    void setRadPerSec(double radPerSec_input){
        radPerSec = radPerSec_input;
    }

    double getTorque(){ //input is radians per second
        torque = voltage * Constants.STALL_TORQUE.getDouble() * (1 - radPerSec / Constants.FREE_SPEED.getDouble());
        return torque;
    }


}