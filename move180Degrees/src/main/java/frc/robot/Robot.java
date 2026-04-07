package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    // --- Constants ---
    private static final int    DEVICE_ID          = 2;
    private static final double GEAR_RATIO         = 45.0;
    private static final double TARGET_ROTATIONS   = (180.0 / 360.0) * GEAR_RATIO; // 22.5

    private static final double kP                 = 0.5;
    private static final double kI                 = 0.0;
    private static final double kD                 = 0.1;
    private static final double kMinOutput         = -1.0;
    private static final double kMaxOutput         =  1.0;
    private static final double POSITION_TOLERANCE = 0.5; // rotations (~4°)

    // --- Hardware ---
    private SparkMax                  motor;
    private SparkClosedLoopController pid;
    private RelativeEncoder           encoder;
    private XboxController            controller;

    @Override
    public void robotInit() {
        motor = new SparkMax(DEVICE_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(kMinOutput, kMaxOutput);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
        encoder.setPosition(0);

        pid = motor.getClosedLoopController();

        controller = new XboxController(0);
    }

    @Override
    public void teleopPeriodic() {
        // A → move to 180°
        if (controller.getAButton()) {
            pid.setSetpoint(TARGET_ROTATIONS, ControlType.kPosition);
        }

        // B → return to 0°
        if (controller.getBButton()) {
            pid.setSetpoint(0, ControlType.kPosition);
        }

        // Y → re-zero encoder at current position
        if (controller.getYButtonPressed()) {
            encoder.setPosition(0);
            pid.setSetpoint(0, ControlType.kPosition);
        }

        SmartDashboard.putNumber("Arm/PositionRot", encoder.getPosition());
        SmartDashboard.putNumber("Arm/PositionDeg", (encoder.getPosition() / GEAR_RATIO) * 360.0);
        SmartDashboard.putBoolean("Arm/AtGoal",
                Math.abs(encoder.getPosition() - TARGET_ROTATIONS) < POSITION_TOLERANCE);
    }

    @Override
    public void disabledInit() {
        motor.stopMotor();
    }
}