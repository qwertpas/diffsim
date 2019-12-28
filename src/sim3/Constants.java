package sim3;

import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextField;


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

    static Constant ROBOT_MASS = new Constant("ROBOT_MASS", 10, Type.DOUBLE); //22 lbs, converted to kg
    static Constant ROBOT_WIDTH = new Constant("ROBOT_WIDTH", 0.46, Type.DOUBLE); //18 in, converted to m
    static Constant DIST_BETWEEN_WHEELS = new Constant("DIST_BETWEEN_WHEELS", 0.4, Type.DOUBLE); //16 in, converted to m
    static Constant WHEEL_RADIUS = new Constant("WHEEL_RADIUS", 0.0635, Type.DOUBLE); //2.5 in, converted to m

    //Overall makes motors slower. Is a torque.
    static Constant GEAR_STATIC_FRIC = new Constant("GEAR_STATIC_FRIC", 0.9, Type.DOUBLE); //actual torque against gearbox when not moving, not the coefficient 
    static Constant GEAR_KINE_FRIC = new Constant("GEAR_KINE_FRIC", 0.5, Type.DOUBLE); //actual torque against gearbox when not moving, not the coefficient 
    static Constant GEAR_FRIC_THRESHOLD = new Constant("GEAR_FRIC_THRESHOLD", 0.001, Type.DOUBLE); //lowest motor speed in rad/sec considered as 'moving' to kine fric

    // Friction between wheels and ground. This stops strafing if the modules are not aligned. Is a coefficient.
    static Constant WHEEL_STATIC_COEFF = new Constant("WHEEL_STATIC_COEFF", 1, Type.DOUBLE); 
    static Constant WHEEL_KINE_COEFF = new Constant("WHEEL_KINE_COEFF", 0.9, Type.DOUBLE); 
    static Constant WHEEL_FRIC_THRESHOLD = new Constant("WHEEL_FRIC_THRESHOLD", 0.001, Type.DOUBLE); //lowest wheel scrub m/sec considered as 'moving' to kine fric

    // Rotational friction that slows robot turning. Is a combo of robot length/width, drop center, friction in omnis. Is a torque.
    static Constant ROT_STATIC_FRIC = new Constant("ROT_STATIC_FRIC", 5, Type.DOUBLE); 
    static Constant ROT_KINE_FRIC = new Constant("ROT_KINE_FRIC", 5, Type.DOUBLE); 
    static Constant ROT_FRIC_THRESHOLD = new Constant("ROT_FRIC_THRESHOLD", 0.001, Type.DOUBLE); //lowest robot angular velocity rad/sec considered as 'moving' to kine fric

    static Constant GRAV_ACCEL = new Constant("GRAV_ACCEL", 9.81, Type.DOUBLE);

    static Constant CONTROLLER_INDEX = new Constant("CONTROLLER_INDEX", 0, Type.INT); //which joystick?

    static Constant DISPLAY_SCALE = new Constant("DISPLAY_SCALE", 100, Type.DOUBLE); //in pixels per meter

    //constants that are editable by GraphicInput
    static Constant[] constants = {GEAR_RATIO, 
                                   MODULE_ROT_INERTIA,
                                   ROBOT_MASS, 
                                   ROBOT_WIDTH, 
                                   DIST_BETWEEN_WHEELS,
                                   WHEEL_RADIUS,
                                   GEAR_STATIC_FRIC,
                                   GEAR_KINE_FRIC,
                                   GEAR_FRIC_THRESHOLD,
                                   WHEEL_STATIC_COEFF,
                                   WHEEL_KINE_COEFF,
                                   WHEEL_FRIC_THRESHOLD,
                                   ROT_STATIC_FRIC,
                                   ROT_KINE_FRIC,
                                   ROT_FRIC_THRESHOLD,
                                   GRAV_ACCEL,
                                   CONTROLLER_INDEX,
                                   DISPLAY_SCALE,
                                  };

    /** ////////////////////////////////
     * CALCULATED FROM REAL CONSTANTS
     * //////////////////////////////// */     
    
    
    
    static double HALF_DIST_BETWEEN_WHEELS = DIST_BETWEEN_WHEELS.getDouble() / 2.0;

    static double WHEEL_STATIC_FRIC = ROBOT_MASS.getDouble() * GRAV_ACCEL.getDouble() * WHEEL_STATIC_COEFF.getDouble(); //is a torque
    static double WHEEL_KINE_FRIC = ROBOT_MASS.getDouble() * GRAV_ACCEL.getDouble() * WHEEL_KINE_COEFF.getDouble(); //is a torque

    static double ROBOT_ROT_INERTIA = (1.0/6.0) * ROBOT_MASS.getDouble() * ROBOT_WIDTH.getDouble() * ROBOT_WIDTH.getDouble();
    // static double ROBOT_ROT_INERTIA = 2;
    //https://en.wikipedia.org/wiki/List_of_moments_of_inertia

    static void calcConstants(){
        HALF_DIST_BETWEEN_WHEELS = DIST_BETWEEN_WHEELS.getDouble() / 2.0;

        WHEEL_STATIC_FRIC = ROBOT_MASS.getDouble() * GRAV_ACCEL.getDouble() * WHEEL_STATIC_COEFF.getDouble(); //is a torque
        WHEEL_KINE_FRIC = ROBOT_MASS.getDouble() * GRAV_ACCEL.getDouble() * WHEEL_KINE_COEFF.getDouble(); //is a torque

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