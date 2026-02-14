package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    // Declare motor, encoder, and controller
    private SparkMax sparkMax;
    private RelativeEncoder encoder;
    private XboxController xboxController;

    // SPARK MAX CAN ID and speed settings
    private static final int SPARK_MAX_CAN_ID = 12; 
    private static final double POSITION_TOLERANCE = 0.001;
    

    // PID constants
    
     private static double kP = 0.05; // Proportional gain
     private static double kI = 0.00; // Integral gain
     private static double kD = 0.002; // Derivative gain
     PIDController pid = new PIDController(kP, kI, kD);
    

    // PID variables
    private double integral = 0.0;
    private double previousError = 0.0;

    // Target position for the motor
    private double targetPosition = 0.0;
    

    @Override
    public void robotInit() {
        // Initialize the SPARK MAX motor controller
        sparkMax = new SparkMax(SPARK_MAX_CAN_ID, MotorType.kBrushless);

        // Initialize the encoder
        encoder = sparkMax.getEncoder();

        // Initialize the Xbox controller
        xboxController = new XboxController(2);
    }

    @Override
    public void teleopPeriodic() {
        /*
         * Important Constants
         * 1 rotation = + or - 1 targetPosition
         * 1 inch = 160 rotations
         */
        // Increment position with Y button
        /* kP = SmartDashboard.getNumber("kP", 0.0); //testing  purposes
        kI = SmartDashboard.getNumber("kI", 0.0);
        kD = SmartDashboard.getNumber("kD", 0.0); */



        // Reset position with X button
        if (xboxController.getXButtonPressed()) {
            targetPosition = 0.0;
        }

        // Reset encoder position to 0 with B button; Zero
        if (xboxController.getBButtonPressed()) {
            encoder.setPosition(0);
            targetPosition = 0.0; // Sync target position
           
        }

        //Decrement position with Y button
        if (xboxController.getYButtonPressed()) {
            targetPosition += 10.0; // Increment by 1 rotation
        }

        // Decrement position with A button
        if (xboxController.getAButtonPressed()) {
            targetPosition -= 160.0; // Decrement by 1 rotation 
        }

        // Make the Linear Actuator move up slightly; Used to help get an exact position
        if (xboxController.getLeftBumperPressed()) {
            targetPosition -= 0.5; // Decrement by 0.5 rotation
        }

        // Make the Linear Actuator move down slightly; Used to help get an exact position
        if (xboxController.getRightBumperPressed()) {
            targetPosition += 0.5; // Increment by 0.5 rotation
        }



        /*
         * PID calculations
         */
        // Get the current position and calculate error
        double currentPosition = encoder.getPosition();
        double error = targetPosition - currentPosition;

        // Calculate the integral and derivative
        integral += error * 0.02; // Assuming teleopPeriodic runs at ~50 Hz (20 ms loop time)
        double derivative = (error - previousError) / 0.02;

        // Calculate PID output
        double pidOutput = (kP * error) + (kI * integral) + (kD * derivative);
    
        // Limit the PID output to the motor speed range
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput)); // Limit between -1.0 and 1.0

        // Set the motor output
        if (Math.abs(error) > POSITION_TOLERANCE) {
            sparkMax.set(pidOutput);
        } else {
            sparkMax.set(0); // Stop the motor if within tolerance
        }

        // Update the previous error
        previousError = error;

        // Print current and target positions for debugging
        System.out.println("Current Position: " + currentPosition);
        System.out.println("Target Position: " + targetPosition);
        SmartDashboard.putNumber("PID Output", pidOutput); //Allows you to see pid Output and the other values
        SmartDashboard.putNumber("kP", kP); 
        SmartDashboard.putNumber("kI", kI);
        SmartDashboard.putNumber("kD", kD);
    }

    @Override
    public void autonomousPeriodic() {
             sparkMax.set(pid.calculate(encoder.getPosition(), targetPosition)); //gets PID in auto
    }

    @Override
    public void disabledInit() {
        // Stop the motor when the robot is disabled
        sparkMax.stopMotor();
    }
}