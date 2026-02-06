
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Robot extends TimedRobot {
    // Create a motor controller object for the NEO motor.
    private SparkMax neoMotor = new SparkMax(6, MotorType.kBrushless);

    // Create a DigitalInput object for the limit switch.
    private DigitalInput limitSwitch = new DigitalInput(0); // Digital channel 
    

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putBoolean("Sensor", limitSwitch.get());
      System.out.println("Limit switch state: " + limitSwitch.get());
        // Check the limit switch.
        // Assuming using a pull-up resistor, so pressed == false (LOW)
        if (limitSwitch.get()) {  
            // The limit switch is pressed.
            
            // Immediately stop the motor.
            
            
            neoMotor.stopMotor();
            
        } else {
            // Otherwise, allow normal operation.
            // For example, set the motor to run at some speed (this could be replaced with your control logic).
            neoMotor.set(0.5);  // 50% power (modify as needed)
        }
    }
}
