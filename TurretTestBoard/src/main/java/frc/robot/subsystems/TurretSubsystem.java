package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

  // ========== LIMIT SWITCH ==========
  private final DigitalInput limitSwitch;
  private final int LIMIT_SWITCH_DIO_PORT = 0; // Change this to your DIO port
  private boolean lastLimitSwitchState = false;
  
  // The angle that the limit switch represents (in degrees)
  private static final double LIMIT_SWITCH_ANGLE = 180.0;

  // ========== MOTOR CONSTANTS ==========
  private final DCMotor dcMotor = DCMotor.getNeo550(1);
  private final int canID = 6; // Change to your CAN ID
  private final double gearRatio = 4; // Change to your gear ratio
  
  // PID Constants - TUNE THESE FOR YOUR ROBOT
  private final double kP = 1.0;
  private final double kI = 0.0;
  private final double kD = 0.0;
  
  // Feedforward Constants - TUNE THESE FOR YOUR ROBOT
  private final double kS = 0.0;
  private final double kV = 0.0;
  private final double kA = 0.0;
  private final double kG = 0.0; // Unused for horizontal turrets
  
  // Motion Constraints
  private final double maxVelocity = 1; // rad/s
  private final double maxAcceleration = 1; // rad/s²
  
  // Motor Configuration
  private final boolean brakeMode = true;
  private final double statorCurrentLimit = 40; // Amps
  
  // Turret angle limits (in degrees)
  private static final double MIN_TURRET_ANGLE = -180.0;
  private static final double MAX_TURRET_ANGLE = 180.0;
  
  // Current desired angle (in degrees)
  private double desiredAngleDegrees = 0.0;

  // Feedforward
  private final ArmFeedforward feedforward = new ArmFeedforward(
    kS, kG, kV, kA
  );

  // Motor controller
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkSim motorSim;
  private final SparkClosedLoopController sparkPidController;

  // Simulation
  private final SingleJointedArmSim turretSim;

  /**
   * Creates a new Turret Subsystem with limit switch.
   */
  public TurretSubsystem() {
    // Initialize limit switch
    limitSwitch = new DigitalInput(LIMIT_SWITCH_DIO_PORT);
    
    // Initialize motor controller
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motor = new SparkMax(canID, MotorType.kBrushless);
    motorConfig.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

    // Configure encoder
    encoder = motor.getEncoder();
    encoder.setPosition(0);

    // Set current limits
    motorConfig.smartCurrentLimit((int) statorCurrentLimit);

    // Configure PID and Feedforward
    sparkPidController = motor.getClosedLoopController();
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
    motorConfig.closedLoop.feedForward.kS(kS).kV(kV).kA(kA).kG(kG);

    // Configure Encoder Gear Ratio
    motorConfig.encoder
      .positionConversionFactor(1.0 / gearRatio)
      .velocityConversionFactor((1.0 / gearRatio) / 60.0); // Convert RPM to RPS

    // Save configuration
    motor.configure(
      motorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    motorSim = new SparkSim(motor, dcMotor);

    // Initialize simulation
    turretSim = new SingleJointedArmSim(
      dcMotor,
      gearRatio,
      0.01, // Moment of inertia
      0.1, // Arm length (m)
      Units.degreesToRadians(MIN_TURRET_ANGLE),
      Units.degreesToRadians(MAX_TURRET_ANGLE),
      false, // No gravity for horizontal turret
      Units.degreesToRadians(0)
    );
  }

  /**
   * Update telemetry and handle limit switch.
   */
  @Override
  public void periodic() {
    // Check limit switch
    boolean limitSwitchPressed = !limitSwitch.get(); // Inverted because limit switches are normally open
    System.out.println("current position: " + getPositionDegrees());
    System.out.println("desiredAngle" + desiredAngleDegrees);
    // Detect rising edge (limit switch just pressed)
    if (limitSwitchPressed && !lastLimitSwitchState) {
      resetToLimitSwitch();
      System.out.println("Limit switch hit! Resetting turret to " + LIMIT_SWITCH_ANGLE + " degrees");
    }
    
    lastLimitSwitchState = limitSwitchPressed;
    
    // Get normalized angle for telemetry
    double normalizedAngle = normalizeAngle(Units.radiansToDegrees(getPositionRadians()));
    
    // Telemetry
    SmartDashboard.putNumber("Turret/Current Angle", normalizedAngle);
    SmartDashboard.putNumber("Turret/Current Angle (Raw)", Units.radiansToDegrees(getPositionRadians()));
    SmartDashboard.putNumber("Turret/Desired Angle", desiredAngleDegrees);
    SmartDashboard.putBoolean("Turret/At Setpoint", atSetpoint());
    SmartDashboard.putNumber("Turret/Velocity", getVelocity());
    SmartDashboard.putNumber("Turret/Current", getCurrent());
    SmartDashboard.putNumber("Turret/Voltage", getVoltage());
    SmartDashboard.putBoolean("Turret/Limit Switch", limitSwitchPressed);
  }

  /**
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    turretSim.setInput(getVoltage());

    // Update simulation by 20ms
    turretSim.update(0.020);
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        turretSim.getCurrentDrawAmps()
      )
    );

    double motorPosition = Radians.of(turretSim.getAngleRads() * gearRatio).in(Rotations);
    double motorVelocity = RadiansPerSecond.of(
      turretSim.getVelocityRadPerSec() * gearRatio
    ).in(RotationsPerSecond);
    motorSim.iterate(motorVelocity, RoboRioSim.getVInVoltage(), 0.02);
  }

  // ========== LIMIT SWITCH METHODS ==========
  
  /**
   * Reset the encoder position when the limit switch is hit.
   */
  private void resetToLimitSwitch() {
    double limitPositionRadians = Units.degreesToRadians(LIMIT_SWITCH_ANGLE);
    double limitPositionRotations = limitPositionRadians / (2.0 * Math.PI);
    encoder.setPosition(limitPositionRotations);
    
    // Stop the motor
    setVelocity(0);
  }
  
  /**
   * Check if the limit switch is pressed.
   * @return True if limit switch is pressed
   */
  public boolean isLimitSwitchPressed() {
    return !limitSwitch.get(); // Inverted
  }

  // ========== POSITION AND VELOCITY GETTERS ==========

  /**
   * Get the current position in Rotations.
   * @return Position in Rotations
   */
  @Logged(name = "Position/Rotations")
  public double getPosition() {
    return encoder.getPosition();
  }
  
  /**
   * Get the current position in Radians.
   * @return Position in Radians
   */
  public double getPositionRadians() {
    return encoder.getPosition() * 2.0 * Math.PI;
  }
  
  /**
   * Get the current position in Degrees.
   * @return Position in Degrees
   */
  public double getPositionDegrees() {
    return Units.radiansToDegrees(getPositionRadians());
  }

  /**
   * Get the current velocity in rotations per second.
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity")
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  /**
   * Get the current motor current.
   * @return Motor current in amps
   */
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  // ========== CONTROL METHODS ==========

  /**
   * Set turret angle.
   * @param angleDegrees The target angle in degrees
   */
  public void setAngle(double angleDegrees) {
    setAngle(angleDegrees, 0);
  }

  /**
   * Set turret angle with acceleration.
   * @param angleDegrees The target angle in degrees
   * @param acceleration The acceleration in rad/s²
   */
  public void setAngle(double angleDegrees, double acceleration) {
    // Clamp to limits
    angleDegrees = normalizeAngle(angleDegrees);
    angleDegrees = MathUtil.clamp(angleDegrees, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
    
    // Check if we're trying to move past the limit switch
    if (angleDegrees < LIMIT_SWITCH_ANGLE && isLimitSwitchPressed()) {
      angleDegrees = LIMIT_SWITCH_ANGLE;
    }
    
    desiredAngleDegrees = angleDegrees;
    
    // Convert degrees to rotations
    
    double angleRadians = Units.degreesToRadians(angleDegrees);
    double positionRotations = angleRadians / (2.0 * Math.PI);

    System.out.println("angle is" + angleDegrees);    
    
    System.out.println("posotion setpoint is " +positionRotations );
    sparkPidController.setSetpoint(
      positionRotations,
      ControlType.kMAXMotionPositionControl,
      ClosedLoopSlot.kSlot0
    );
  }

  /**
   * Set turret angular velocity.
   * @param velocityDegPerSec The target velocity in degrees per second
   */
  public void setVelocity(double velocityDegPerSec) {
    setVelocity(velocityDegPerSec, 0);
  }

  /**
   * Set turret angular velocity with acceleration.
   * Respects limit switch and angle limits.
   * @param velocityDegPerSec The target velocity in degrees per second
   * @param acceleration The acceleration in degrees per second squared
   */
  public void setVelocity(double velocityDegPerSec, double acceleration) {
    // Normalize current angle to -180 to 180
    double currentAngle = normalizeAngle(getPositionDegrees());
    
    // Check if we're at or past limits
    boolean atMinLimit = currentAngle <= MIN_TURRET_ANGLE;
    boolean atMaxLimit = currentAngle >= MAX_TURRET_ANGLE;
    
    // Block movement if we're at a limit and trying to go further
    if ((atMinLimit && velocityDegPerSec < 0) || (atMaxLimit && velocityDegPerSec > 0)) {
      velocityDegPerSec = 0;
    }
    
    // Also check limit switch
    if (velocityDegPerSec < 0 && isLimitSwitchPressed()) {
      velocityDegPerSec = 0;
    }
    
    // Convert degrees/sec to rotations/sec
    double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
    double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);

    sparkPidController.setSetpoint(
      velocityRotations,
      ControlType.kVelocity,
      ClosedLoopSlot.kSlot0
    );
  }

  /**
   * Set motor voltage directly.
   * Respects limit switch and angle limits.
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    // Normalize current angle to -180 to 180
    double currentAngle = normalizeAngle(getPositionDegrees());
    
    // Block negative voltage (moving toward limit) if at limit switch
    if (voltage < 0 && isLimitSwitchPressed()) {
      voltage = 0;
    }
    
    // Block movement at normalized angle limits
    if (voltage < 0 && currentAngle <= MIN_TURRET_ANGLE) {
      voltage = 0;
    }
    if (voltage > 0 && currentAngle >= MAX_TURRET_ANGLE) {
      voltage = 0;
    }
    
    motor.setVoltage(voltage);
  }
  
  /**
   * Stop the turret.
   */
  public void stop() {
    setVelocity(0);
  }

  /**
   * Check if turret is at the desired angle.
   * Uses normalized angles for comparison.
   * @param tolerance Tolerance in degrees
   * @return True if within tolerance
   */
  public boolean atSetpoint(double tolerance) {
    double currentAngle = normalizeAngle(Units.radiansToDegrees(getPositionRadians()));
    double normalizedDesired = normalizeAngle(desiredAngleDegrees);
    return Math.abs(normalizeAngle(currentAngle - normalizedDesired)) < tolerance;
  }
  
  /**
   * Check if turret is at the desired angle with default tolerance.
   * @return True if within 2 degrees
   */
  public boolean atSetpoint() {
    return atSetpoint(2.0);
  }

  /**
   * Normalize an angle to the range -180 to 180 degrees.
   * @param angle Angle in degrees
   * @return Normalized angle in degrees
   */
  private double normalizeAngle(double angle) {
    angle = angle % 360.0;
    if (angle > 180.0) {
      angle -= 360.0;
    } else if (angle < -180.0) {
      angle += 360.0;
    }
    return angle;
  }

  // ========== COMMANDS ==========

  /**
   * Creates a command to set the turret to a specific angle.
   * @param angleDegrees The target angle in degrees
   * @return A command that sets the turret to the specified angle
   */
  public Command setAngleCommand(double angleDegrees) {
    return runOnce(() -> setAngle(angleDegrees));
  }

  /**
   * Creates a command to move the turret to a specific angle and wait until reached.
   * @param angleDegrees The target angle in degrees
   * @return A command that moves the turret to the specified angle
   */
  public Command moveToAngleCommand(double angleDegrees) {
    return run(() -> setAngle(angleDegrees))
      .until(() -> atSetpoint(2.0));
  }

  /**
   * Creates a command to stop the turret.
   * @return A command that stops the turret
   */
  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  /**
   * Creates a command to move the turret at a specific velocity.
   * @param velocityDegPerSec The target velocity in degrees per second
   * @return A command that moves the turret at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityDegPerSec) {
    return run(() -> setVelocity(velocityDegPerSec));
  }
  
  /**
   * Creates a command for manual control with a joystick/controller.
   * @param speedSupplier Supplier that returns speed [-1, 1]
   * @param maxSpeed Maximum speed in degrees per second
   * @return A command for manual control
   */
  public Command manualControlCommand(java.util.function.DoubleSupplier speedSupplier, double maxSpeed) {
    return run(() -> {
      double speed = speedSupplier.getAsDouble();
      
      // Apply deadband
      if (Math.abs(speed) < 0.1) {
        speed = 0;
      }
      
      // Scale to max speed
      double velocity = speed * maxSpeed;
      
      setVelocity(velocity);
    });
  }

  /**
   * Get the turret simulation for testing.
   * @return The turret simulation model
   */
  public SingleJointedArmSim getSimulation() {
    return turretSim;
  }
}