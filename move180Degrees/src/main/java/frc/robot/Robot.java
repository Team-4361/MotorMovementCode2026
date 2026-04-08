package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

    private static final int    LEFT_MOTOR_CAN_ID  = 1;
    private static final int    RIGHT_MOTOR_CAN_ID = 2;
    private static final int    XBOX_PORT          = 0;
    private static final double MAX_SPEED          = 0.10;

    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private XboxController xbox;

    @Override
    public void robotInit() {
        leftMotor  = new SparkMax(LEFT_MOTOR_CAN_ID,  MotorType.kBrushless);
        rightMotor = new SparkMax(RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .inverted(true);

        leftMotor.configure(leftConfig,   ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        xbox = new XboxController(XBOX_PORT);
    }

    @Override
    public void teleopPeriodic() {
        double rawInput = -xbox.getLeftY();
        double speed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, rawInput * MAX_SPEED));

        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    @Override
    public void disabledInit() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
}