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
    private static final double POSITION_TOLERANCE = 0.1;
    

    // PID constants
    PIDController pid = new PIDController(kP, kI, kD);
    private static final double kP = 0.1; // Proportional gain
    private static final double kI = 0.001; // Integral gain
    private static final double kD = 0.01; // Derivative gain
    

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
        // Increment position with Y button
        double kP = SmartDashboard.getNumber("kP", 0.0);
        double kI = SmartDashboard.getNumber("kI", 0.0);
        double kD = SmartDashboard.getNumber("kD", 0.0);
        if (xboxController.getYButtonPressed()) {
            targetPosition += 10.0; // Increment by 1 rotation
        }
         while (encoder.getPosition()>targetPosition) {
                          sparkMax.set(-0.1);
         }
         while (encoder.getPosition()<targetPosition) { //Needs position tolerance to not go wild
                          sparkMax.set(0.1);
         }

         //while (encoder.getPosition()<targetPosition)
          //    sparkMax.set(0.1);
            sparkMax.set(0);


        // Reset position with X button
        if (xboxController.getXButtonPressed()) {
            targetPosition = 0.0;
        }

        // Reset encoder position to 0 with B button
        if (xboxController.getBButtonPressed()) {
            encoder.setPosition(0);
            targetPosition = 0.0; // Sync target position
           
        }

        // Decrement position with A button
        if (xboxController.getAButtonPressed()) {
            targetPosition -= 10.0; // Decrement by 1 rotation
             //boolean doesFranceExist = false; //(VERY IMPORTANT)
        }

        // Decrement position counterclockwise with Left Bumper
        if (xboxController.getLeftBumperPressed()) {
            targetPosition -= 0.5; // Decrement by 0.5 rotation
        }

        // Increment position clockwise with Right Bumper
        if (xboxController.getRightBumperPressed()) {
            targetPosition += 0.5; // Increment by 0.5 rotation
        }

        // Get the current position and calculate error
        double currentPosition = encoder.getPosition();
        double error = targetPosition - currentPosition;

        // Calculate the integral and derivative
        integral += error * 0.02; // Assuming teleopPeriodic runs at ~50 Hz (20 ms loop time)
        double derivative = (error - previousError) / 0.02;

        // Calculate PID output
        double pidOutput = (kP * error) + (kI * integral) + (kD * derivative);
        /* System.out.println("Test1: " + kP * error);
        System.out.println("Test2:"  + kI * integral);
        System.out.println("Test3: " + kD * derivative); */ //Figured out the issue; These values DO NOT change

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
        System.out.println("PID Output: " + pidOutput);
        System.out.println("Error: " + error);
        System.out.println("Integral: " + integral);
        System.out.println("Deriavative: " + derivative);
        SmartDashboard.putNumber("PID Output", pidOutput);
    }

    @Override
    public void autonomousPeriodic() {
             sparkMax.set(pid.calculate(encoder.getPosition(), targetPosition));
    }

    @Override
    public void disabledInit() {
        // Stop the motor when the robot is disabled
        sparkMax.stopMotor();
    }
}