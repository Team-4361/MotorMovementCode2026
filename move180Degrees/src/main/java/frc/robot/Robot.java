package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    // --- Constants ---
    private static final int    DEVICE_ID          = 2;
    private static final double TARGET_POSITION    = 0.5;   // 180° = 0.5 on a 0.0–1.0 absolute scale
    private static final double ZERO_OFFSET        = 0.7151296;   // tune this to your arm's physical zero
    private static final double kP                 = 0.5;
    private static final double kI                 = 0.0;
    private static final double kD                 = 0.1;
    private static final double kMinOutput         = -1.0;
    private static final double kMaxOutput         =  1.0;
    private static final double POSITION_TOLERANCE = 0.01;  // ~3.6° on 0–1 scale

    // --- Hardware ---
    private SparkMax                  motor;
    private SparkClosedLoopController pid;
    private AbsoluteEncoder           encoder;
    private XboxController            controller;

    @Override
    public void robotInit() {
        motor = new SparkMax(DEVICE_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        // Absolute encoder config — adjust zeroOffset to align arm's resting position to 0
        AbsoluteEncoderConfig absEncConfig = new AbsoluteEncoderConfig();
        absEncConfig
            .zeroOffset(ZERO_OFFSET)
            .inverted(false);          // flip to true if encoder reads backwards

        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .apply(absEncConfig);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)  // <-- key change
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(kMinOutput, kMaxOutput);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getAbsoluteEncoder();  // <-- pulls from the data port
        pid     = motor.getClosedLoopController();

        controller = new XboxController(0);
    }

    @Override
    public void teleopPeriodic() {
        double currentPos = encoder.getPosition(); // 0.0 – 1.0 (full rotation)

        // A → move to 180°
        if (controller.getAButton()) {
            pid.setReference(TARGET_POSITION, ControlType.kPosition);
        }

        // B → return to 0° (or wherever zeroOffset puts "home")
        if (controller.getBButton()) {
            pid.setReference(0.0, ControlType.kPosition);
        }

        // Y → can't re-zero an absolute encoder in software the same way,
        //     so just drive back to 0 as a safe fallback
        if (controller.getYButtonPressed()) {
            pid.setReference(0.0, ControlType.kPosition);
        }

        SmartDashboard.putNumber("Arm/Position (0-1)",  currentPos);
        SmartDashboard.putNumber("Arm/Position (deg)",  currentPos * 360.0);
        SmartDashboard.putBoolean("Arm/AtGoal",
                Math.abs(currentPos - TARGET_POSITION) < POSITION_TOLERANCE);
    }

    @Override
    public void disabledInit() {
        motor.stopMotor();
    }
}