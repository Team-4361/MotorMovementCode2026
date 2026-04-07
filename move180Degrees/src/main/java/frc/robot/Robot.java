package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    // --- Constants ---
    private static final int    DEVICE_ID          = 2;
    private static final double TARGET_POSITION    = 0.5;   // 180° on 0.0–1.0 scale
    private static final double ZERO_OFFSET        = 0.0;   // tune to your arm's home
    private static final double kP                 = 0.5;
    private static final double kI                 = 0.0;
    private static final double kD                 = 0.1;
    private static final double kMinOutput         = -1.0;
    private static final double kMaxOutput         =  1.0;
    private static final double POSITION_TOLERANCE = 0.01;  // ~3.6°

    // --- Hardware ---
    private SparkMax                  motor;
    private SparkClosedLoopController pid;
    private AbsoluteEncoder           encoder;
    private XboxController            controller;

    @Override
    public void robotInit() {
        motor = new SparkMax(DEVICE_ID, MotorType.kBrushless);

        AbsoluteEncoderConfig absEncConfig = new AbsoluteEncoderConfig();
        absEncConfig
            .zeroOffset(ZERO_OFFSET)
            .inverted(false);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .apply(absEncConfig);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(kMinOutput, kMaxOutput);

        // kNoResetSafeParameters keeps existing CAN config intact,
        // avoids the timeout caused by a full reset on a fresh controller
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        encoder    = motor.getAbsoluteEncoder();
        pid        = motor.getClosedLoopController();
        controller = new XboxController(0);
    }

    @Override
    public void teleopPeriodic() {
        double currentPos = encoder.getPosition();

        // A → move to 180°
        if (controller.getAButton()) {
            // setReference() with ClosedLoopSlot replaces the deprecated setSetpoint()
            pid.setReference(TARGET_POSITION, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        // B → return home
        if (controller.getBButton()) {
            pid.setReference(0.0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        // Y → command back to home (absolute encoder can't be re-zeroed mid-match)
        if (controller.getYButtonPressed()) {
            pid.setReference(0.0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        SmartDashboard.putNumber("Arm/Position (0-1)", currentPos);
        SmartDashboard.putNumber("Arm/Position (deg)",  currentPos * 360.0);
        SmartDashboard.putBoolean("Arm/AtGoal",
                Math.abs(currentPos - TARGET_POSITION) < POSITION_TOLERANCE);
    }

    @Override
    public void disabledInit() {
        motor.stopMotor();
    }
}