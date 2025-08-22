package frc.robot;

public class Constants {
    public static class PIDConstants
    {
        // H - Auto Heading PID
        public static final double HKP = 0.10;   
        public static final double HKD = 0.0;   
        public static final double HKI = 0.1;   
        // V - Velocity PID for Roller
        public static final double VKP = 0.05;
        public static final double VKI = 0;
        public static final double VKD = 0;
        public static final double VKFF = 1.0 / 5676.0;  // ≈ 0.0001762
    }
    public static class OperatorConstants 
    {
        public static int DRIVER_CONTROLLER_PORT = 0;
        public static int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static class ClimberConstants
    {
        public static int CLIMB_MOTOR = 5; 
        public static int ClIMB_MOTOR_CURRENT_LIMIT = 60; 
        public static double CLIMB_MOTOR_VOLT_COMP = 12; 
        public static double CLIMBER_SPEED_DOWN = -0.5;
        public static double CLIMBER_SPEED_UP = 0.5;
    }
    public static class ArmConstants 
    {
        public static int ARM_MOTOR = 6; 
        public static int ARM_MOTOR_CURRENT_LIMIT = 60; 
        public static final double ARM_MOTOR_VOLTAGE_COMP = 12; 
        public static final double ARM_SPEED = 0.5; 
    }


    public static class RollerConstants
    {
        public static final int ROLLER_MOTOR_ID = 7; 
        public static final int ROLLER_MOTOR_CURRENT_LIMIT = 30; 
        public static final double ROLLER_MOTOR_VOLTAGE_COMP = 12; 
        public static final double CLIMBER_MOTOR_SPEED_DOWN = 12;
        public static final double CLIMBER_SPEED_DOWN = 0.5; 
        public static final double VELOCITY_RPM = 3000;
    }
}   
