package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    // --- Constants ---
    private static final int    DEVICE_ID          = 1;
    private static final double GEAR_RATIO         = 45.0;
    private static final double TARGET_ROTATIONS   = (180.0 / 360.0) * GEAR_RATIO; // 22.5

    private static final double kP                 = 0.1;
    private static final double kI                 = 0.0;
    private static final double kD                 = 0.0;
    private static final double POSITION_TOLERANCE = 0.1; // rotations (~4°)

    // --- Hardware ---
    private SparkMax        motor;
    private RelativeEncoder encoder;
    private XboxController  controller;

    // --- WPILib PID ---
    private PIDController   pid;
    private double          setpoint = 0;

    @Override
    public void robotInit() {
        motor = new SparkMax(DEVICE_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
        encoder.setPosition(0);

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(POSITION_TOLERANCE);

        controller = new XboxController(0);
    }

    @Override
    public void teleopPeriodic() {
        // A → move to 180°
        if (controller.getAButton()) {
            setpoint = TARGET_ROTATIONS;
        }

        // B → return to 0°
        if (controller.getBButton()) {
            setpoint = 0;
        }

        // Y → re-zero encoder
        if (controller.getYButtonPressed()) {
            encoder.setPosition(0);
            setpoint = 0;
            pid.reset();
        }

        // Calculate PID output and feed to motor
        double output = pid.calculate(encoder.getPosition(), setpoint);
        output = Math.max(-0.5, Math.min(0.5, output)); // clamp to ±50%
        motor.set(output);

        SmartDashboard.putNumber("Arm/PositionRot", encoder.getPosition());
        SmartDashboard.putNumber("Arm/PositionDeg", (encoder.getPosition() / GEAR_RATIO) * 360.0);
        SmartDashboard.putNumber("Arm/Output",      output);
        SmartDashboard.putBoolean("Arm/AtGoal",     pid.atSetpoint());
    }

    @Override
    public void disabledInit() {
        motor.stopMotor();
        pid.reset();
    }
}