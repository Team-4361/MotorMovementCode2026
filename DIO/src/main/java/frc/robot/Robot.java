package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    // DIO port 0
    private DigitalInput dio0;

    @Override
    public void robotInit() {
        dio0 = new DigitalInput(0);
    }

    @Override
    public void robotPeriodic() {
        boolean dioState = dio0.get(); // true = HIGH, false = LOW
        System.out.println("DIO 0 state: " + dioState);
    }
}
