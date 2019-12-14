package sim3;

import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextField;

import sim3.Util;



public class Constants{

    enum Type{
        BOOLEAN, INT, DOUBLE, STRING;
    }

    JPanel panel = new JPanel();

    /** ////////////////////////////////////////////
     * DISPLAY PREFERENCES
     * //////////////////////////////////////////// */ 
    static Boolean printPowers = false;
    

    /** ////////////////////////////////////////////
     * REAL PHYSICAL CONSTANTS (meters, kilograms, seconds) that come from GraphicInput
     * //////////////////////////////////////////// */ 


    static Constant STALL_TORQUE = new Constant("STALL_TORQUE", 0.537, Type.DOUBLE); //76 oz-in for yellowjacket 3.7, converted to newton*meters
    static Constant FREE_SPEED = new Constant("FREE_SPEED", 169.6, Type.DOUBLE); //1620 rpm for yellowjacket 3.7, converted to rad/sec
    static Constant GEAR_RATIO = new Constant("GEAR_RATIO", 4, Type.DOUBLE);
    static Constant MODULE_ROT_INERTIA = new Constant("MODULE_ROT_INERTIA", 0.0005, Type.DOUBLE); //Assume a cylinder with radius 0.1m mass 0.1kg

    static Constant ROBOT_MASS = new Constant("ROBOT_MASS", 10, Type.DOUBLE);
    static Constant ROBOT_WIDTH = new Constant("ROBOT_WIDTH", 0.46, Type.DOUBLE);
    static Constant DIST_BETWEEN_WHEELS = new Constant("DIST_BETWEEN_WHEELS", 0.4, Type.DOUBLE);
    static Constant WHEEL_RADIUS = new Constant("WHEEL_RADIUS", 0.0762, Type.DOUBLE);
    static Constant STATIC_FRIC_COEFF = new Constant("STATIC_FRIC_COEFF", 1.1, Type.DOUBLE); //between wheels and ground
    static Constant KINE_FRIC_COEFF = new Constant("KINE_FRIC_COEFF", 0.7, Type.DOUBLE); //should be < static

    //Overall makes motors slower
    static Constant GEAR_STATIC_FRIC = new Constant("GEAR_STATIC_FRIC", 0.9, Type.DOUBLE); //actual torque against gearbox when not moving, not the coefficient 
    static Constant GEAR_KINE_FRIC = new Constant("GEAR_KINE_FRIC", 0.5, Type.DOUBLE); //actual torque against gearbox when not moving, not the coefficient 
    static Constant GEAR_FRIC_THRESHOLD = new Constant("GEAR_FRIC_THRESHOLD", 0.01, Type.DOUBLE); //lowest motor speed in rad/sec considered as 'moving' to kine fric

    // Wheel scrub torque slows turning, coeff is a combo of fric coeff, drop center, robot length.
    static Constant WHEEL_SCRUB_STATIC_COEFF = new Constant("WHEEL_SCRUB_STATIC_COEFF", 0.012, Type.DOUBLE); 
    static Constant WHEEL_SCRUB_KINE_COEFF = new Constant("WHEEL_SCRUB_KINE_COEFF", 0.01, Type.DOUBLE); 
    static Constant WHEEL_SCRUB_FRIC_THRESHOLD = new Constant("WHEEL_SCRUB_FRIC_THRESHOLD", 0.01, Type.DOUBLE); //lowest robot rad/sec considered as 'moving' to kine fric

    static Constant GRAV_ACCEL = new Constant("GRAV_ACCEL", 9.81, Type.DOUBLE);

    // static Constant CIM_STALL_TORQUE = new Constant("CIM_STALL_TORQUE", 2.413);
    static double CIM_STALL_TORQUE = 2.413;
    static double CIM_TORQUE_SLOPE = -0.0004527;
    static double MINICIM_STALL_TORQUE = 1.409;
    static double MINICIM_TORQUE_SLOPE = -0.0002413;
    static double REDLINE_STALL_TORQUE = 0.7080;
    static double REDLINE_TORQUE_SLOPE = -3.779e-05;

    static Constant CONTROLLER_INDEX = new Constant("Controller_INDEX", 0, Type.INT); //which joystick?

    static Constant DISPLAY_SCALE = new Constant("DISPLAY_SCALE", 100, Type.DOUBLE); //in pixels per meter

    //constants that are editable by GraphicInput
    static Constant[] constants = {GEAR_RATIO, 
                                   MODULE_ROT_INERTIA,
                                   ROBOT_MASS, 
                                   ROBOT_WIDTH, 
                                   DIST_BETWEEN_WHEELS,
                                   WHEEL_RADIUS,
                                   STATIC_FRIC_COEFF,
                                   KINE_FRIC_COEFF,
                                   GEAR_STATIC_FRIC,
                                   GEAR_KINE_FRIC,
                                   GEAR_FRIC_THRESHOLD,
                                   WHEEL_SCRUB_STATIC_COEFF,
                                   WHEEL_SCRUB_KINE_COEFF,
                                   WHEEL_SCRUB_FRIC_THRESHOLD,
                                   GRAV_ACCEL,
                                   CONTROLLER_INDEX,
                                   DISPLAY_SCALE,
                                  };

    /** ////////////////////////////////
     * CALCULATED FROM REAL CONSTANTS
     * //////////////////////////////// */     
    
    
    
    static double HALF_DIST_BETWEEN_WHEELS = DIST_BETWEEN_WHEELS.getDouble() / 2.0;
    static double STATIC_FRIC = ROBOT_MASS.getDouble() * GRAV_ACCEL.getDouble() * STATIC_FRIC_COEFF.getDouble();
    static double KINE_FRIC = ROBOT_MASS.getDouble() * GRAV_ACCEL.getDouble() * KINE_FRIC_COEFF.getDouble();

    static double WHEEL_SCRUB_STATIC = ROBOT_MASS.getDouble() * GRAV_ACCEL.getDouble() * WHEEL_SCRUB_STATIC_COEFF.getDouble(); //is a torque
    static double WHEEL_SCRUB_KINE = ROBOT_MASS.getDouble() * GRAV_ACCEL.getDouble() * WHEEL_SCRUB_KINE_COEFF.getDouble(); //is a torque

    static double ROBOT_ROT_INERTIA = (1.0/6.0) * ROBOT_MASS.getDouble() * ROBOT_WIDTH.getDouble() * ROBOT_WIDTH.getDouble();
    // static double ROBOT_ROT_INERTIA = 2;
    //https://en.wikipedia.org/wiki/List_of_moments_of_inertia

    static void calcConstants(){
        HALF_DIST_BETWEEN_WHEELS = DIST_BETWEEN_WHEELS.getDouble() / 2.0;
        STATIC_FRIC = ROBOT_MASS.getDouble() * GRAV_ACCEL.getDouble() * STATIC_FRIC_COEFF.getDouble();
        KINE_FRIC = ROBOT_MASS.getDouble() * GRAV_ACCEL.getDouble() * KINE_FRIC_COEFF.getDouble();

        WHEEL_SCRUB_STATIC = ROBOT_MASS.getDouble() * GRAV_ACCEL.getDouble() * WHEEL_SCRUB_STATIC_COEFF.getDouble(); //is a torque
        WHEEL_SCRUB_KINE = ROBOT_MASS.getDouble() * GRAV_ACCEL.getDouble() * WHEEL_SCRUB_KINE_COEFF.getDouble(); //is a torque

        ROBOT_ROT_INERTIA = (1.0/6.0) * ROBOT_MASS.getDouble() * ROBOT_WIDTH.getDouble() * ROBOT_WIDTH.getDouble();
    }

    static Boolean checkTypes(){
        Boolean good = true;
        for(Constant constant : constants){
            try{
                if(constant.type.equals(Type.DOUBLE)) constant.getDouble();
                if(constant.type.equals(Type.INT)) constant.getInt();
                if(constant.type.equals(Type.STRING)) constant.getString();
            }catch(IllegalArgumentException e){
                JOptionPane.showMessageDialog(GraphicInput.panel, constant.name + " must be of type " + constant.type.name());
                good = false;
            }
        }
        return good;
    }

    static void setAllToDefault(){
        for(Constant constant : constants){
            constant.setValue(constant.defaultValue);
        }
    }


    public static class Constant{
        private String name;
        private Object value;
        private String defaultValue;
        
        Type type;
        
        JLabel label;
        JTextField field;


        Constant(String name_input, Object value_input, Type type_input){
            name = name_input;
            value = value_input;
            defaultValue = String.valueOf(value_input);
            type = type_input;
            
            label = new JLabel(name);
            field = new JTextField(String.valueOf(value));
        }
        
        public String getName(){
            return name;
        }

        public double getDouble() {
            return Double.valueOf(getString());
        }

        public int getInt() {
            return Integer.valueOf(getString());
        }

        public String getString() {
            return String.valueOf(value);
        }

        public String getDefaultString() {
            return String.valueOf(defaultValue);
        }

        public Object getObject(){
            return value;
        }
        
        public void setName(String name) {
            this.name = name;
        }
        
        public void setValue(Object value_input) {
            this.value = value_input;
        }

    }

}