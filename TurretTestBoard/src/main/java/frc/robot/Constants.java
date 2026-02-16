package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {
  
  /**
   * Turret subsystem constants
   */
  public static class TurretConstants {
    // ========== HARDWARE CONFIGURATION ==========
    public static final int MOTOR_CAN_ID = 6;
    public static final int LIMIT_SWITCH_DIO_PORT = 0;
    
    // ========== MECHANICAL CONFIGURATION ==========
    public static final double GEAR_RATIO = 4.0; // Motor rotations : Turret rotations
    
    // ========== PID CONSTANTS ==========
    // Start with these values and tune for your robot
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
    // ========== FEEDFORWARD CONSTANTS ==========
    // Characterize your turret to find these values
    public static final double kS = 0.0; // Volts
    public static final double kV = 0.0; // Volts per (radian/second)
    public static final double kA = 0.0; // Volts per (radian/second²)
    public static final double kG = 0.0; // Volts (unused for horizontal turret)
    
    // ========== MOTION CONSTRAINTS ==========
    public static final double MAX_VELOCITY_RAD_PER_SEC = 1.0; // radians/second
    public static final double MAX_ACCELERATION_RAD_PER_SEC_SQ = 1.0; // radians/second²
    
    // ========== CURRENT LIMITS ==========
    public static final int STATOR_CURRENT_LIMIT = 40; // Amps
    
    // ========== ANGLE LIMITS ==========
    public static final double MIN_ANGLE_DEGREES = -180.0;
    public static final double MAX_ANGLE_DEGREES = 180.0;
    public static final double LIMIT_SWITCH_ANGLE_DEGREES = -180.0; // Where the limit switch is positioned
    
    // ========== CONTROL TOLERANCES ==========
    public static final double POSITION_TOLERANCE_DEGREES = 2.0;
    
    // ========== MOTOR CONFIGURATION ==========
    public static final boolean BRAKE_MODE = true;
  }
  
  /**
   * Operator Interface constants
   */
  public static class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    
    // Manual control speeds (degrees per second)
    public static final double MANUAL_TURRET_SPEED = 90.0;
    public static final double FINE_CONTROL_MULTIPLIER = 0.3; // 30% speed
    public static final double FAST_CONTROL_MULTIPLIER = 2.0; // 200% speed
    
    // Joystick deadband
    public static final double JOYSTICK_DEADBAND = 0.1;
    
    // Preset angles
    public static final double PRESET_ANGLE_FORWARD = 0.0;
    public static final double PRESET_ANGLE_LEFT = 90.0;
    public static final double PRESET_ANGLE_RIGHT = -90.0;
    public static final double PRESET_ANGLE_BACK = 180.0;
    
    // D-Pad angle increments
    public static final double SMALL_ANGLE_INCREMENT = 5.0; // degrees
    public static final double LARGE_ANGLE_INCREMENT = 15.0; // degrees
  }
}