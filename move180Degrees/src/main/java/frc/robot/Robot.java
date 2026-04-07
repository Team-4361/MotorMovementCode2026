package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    // --- Constants ---
    private static final int    DEVICE_ID          = 2;
    private static final int    DIO_PORT           = 0;
    private static final double TARGET_POSITION    = 0.5;   // 180° on 0.0–1.0 scale
    private static final double kP                 = 0.1;   // tuned for 50Hz RIO loop
    private static final double kI                 = 0.0;
    private static final double kD                 = 0.01;
    private static final double POSITION_TOLERANCE = 0.01;  // ~3.6°

    // --- Hardware ---
    private SparkMax         motor;
    private DutyCycleEncoder encoder;
    private PIDController    pid;
    private XboxController   controller;

    // Offset applied manually since setPositionOffset() was removed in 2025
    private double zeroOffset = 0.0;

    @Override
    public void robotInit() {
        motor = new SparkMax(DEVICE_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        encoder = new DutyCycleEncoder(DIO_PORT);

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(POSITION_TOLERANCE);

        // Capture the arm's current physical position as zero on startup
        zeroOffset = encoder.get();
        pid.setSetpoint(0.0);

        controller = new XboxController(0);
    }

    // Returns position relative to zero, clamped to 0.0–1.0
    private double getPosition() {
        double pos = encoder.get() - zeroOffset;
        if (pos < 0.0) pos += 1.0;
        if (pos > 1.0) pos -= 1.0;
        return pos;
    }

    @Override
    public void teleopPeriodic() {
        double currentPos = getPosition();

        // A → move to 180°
        if (controller.getAButton()) {
            pid.setSetpoint(TARGET_POSITION);
        }

        // B → return home
        if (controller.getBButton()) {
            pid.setSetpoint(0.0);
        }

        // Y → re-zero at current position
        if (controller.getYButtonPressed()) {
            zeroOffset = encoder.get();
            pid.setSetpoint(0.0);
            pid.reset();
        }

        double output = pid.calculate(currentPos);
        output = Math.max(-1.0, Math.min(1.0, output));
        motor.set(output);

        SmartDashboard.putNumber("Arm/Position (0-1)", currentPos);
        SmartDashboard.putNumber("Arm/Position (deg)",  currentPos * 360.0);
        SmartDashboard.putBoolean("Arm/AtGoal",         pid.atSetpoint());
        SmartDashboard.putNumber("Arm/PID Output",      output);
        SmartDashboard.putNumber("Arm/Raw Encoder",     encoder.get());
    }

    @Override
    public void disabledInit() {
        motor.stopMotor();
        pid.reset();
    }
}