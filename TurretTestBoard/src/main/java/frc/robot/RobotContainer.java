package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Constants for manual control
  private static final double MANUAL_TURRET_SPEED = 90.0; // degrees per second
  private static final double PRESET_ANGLE_FORWARD = 0.0;
  private static final double PRESET_ANGLE_LEFT = 90.0;
  private static final double PRESET_ANGLE_RIGHT = -90.0;
  private static final double PRESET_ANGLE_BACK = 180.0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureBindings();
    
    // Set default commands
    configureDefaultCommands();
  }

  /**
   * Configure default commands for subsystems.
   */
  private void configureDefaultCommands() {
    // Manual turret control using right stick X-axis
    // Positive X = right, Negative X = left
    // This will continuously run and allow the operator to manually control the turret
    turretSubsystem.setDefaultCommand(
      turretSubsystem.manualControlCommand(
        () -> -operatorController.getRightX(), // Negative to make right stick right = positive rotation
        MANUAL_TURRET_SPEED
      )
    );
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    // ========== OPERATOR CONTROLS (Xbox Controller) ==========
    
    // A Button - Move to forward position (0 degrees)
    operatorController.a()
      .onTrue(turretSubsystem.moveToAngleCommand(PRESET_ANGLE_FORWARD));
    
    // B Button - Move to right position (-90 degrees)
    operatorController.b()
      .onTrue(turretSubsystem.moveToAngleCommand(PRESET_ANGLE_RIGHT));
    
    // X Button - Move to left position (90 degrees)
    operatorController.x()
      .onTrue(turretSubsystem.moveToAngleCommand(PRESET_ANGLE_LEFT));
    
    // Y Button - Move to back position (180 degrees)
    operatorController.y()
      .onTrue(turretSubsystem.moveToAngleCommand(PRESET_ANGLE_BACK));
    
    // Left Bumper - Fine control (slower speed)
    operatorController.leftBumper()
      .whileTrue(
        turretSubsystem.manualControlCommand(
          () -> -operatorController.getRightX(),
          MANUAL_TURRET_SPEED * 0.3 // 30% speed for fine control
        )
      );
    
    // Right Bumper - Fast control (faster speed)
    operatorController.rightBumper()
      .whileTrue(
        turretSubsystem.manualControlCommand(
          () -> -operatorController.getRightX(),
          MANUAL_TURRET_SPEED * 2.0 // 200% speed for fast movement
        )
      );
    
    // Back Button - Stop turret immediately
    operatorController.back()
      .onTrue(turretSubsystem.stopCommand());
    
    // Start Button - Reset to limit switch position (-180 degrees)
    // This is useful if the turret gets out of sync
    operatorController.start()
      .onTrue(turretSubsystem.moveToAngleCommand(-180.0));
    
    // D-Pad Controls for precise angle adjustments
    // Up - Increment by 5 degrees
    operatorController.povUp()
      .onTrue(Commands.runOnce(() -> {
        double currentAngle = turretSubsystem.getPositionDegrees();
        turretSubsystem.setAngle(currentAngle + 5.0);
        System.out.println("wow");
      }));
    
    // Down - Decrement by 5 degrees
    operatorController.povDown()
      .onTrue(Commands.runOnce(() -> {
        double currentAngle = turretSubsystem.getPositionDegrees();
        turretSubsystem.setAngle(currentAngle - 5.0);
      }));
    
    // Left - Increment by 15 degrees
    operatorController.povLeft()
      .onTrue(Commands.runOnce(() -> {
        double currentAngle = turretSubsystem.getPositionDegrees();
        turretSubsystem.setAngle(currentAngle + 15.0);
      }));
    
    // Right - Decrement by 15 degrees
    operatorController.povRight()
      .onTrue(Commands.runOnce(() -> {
        double currentAngle = turretSubsystem.getPositionDegrees();
        turretSubsystem.setAngle(currentAngle - 15.0);
      }));

    // ========== DRIVER CONTROLS (Optional) ==========
    // You can add driver controls here if needed
    // For example, quick turret snapshots while driving
    
    // Example: Driver can also control turret with their right stick
    // (Commented out by default - uncomment if you want driver control)
    /*
    driverController.rightBumper()
      .whileTrue(
        turretSubsystem.manualControlCommand(
          () -> -driverController.getRightX(),
          MANUAL_TURRET_SPEED * 0.5
        )
      );
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Example autonomous command: Move turret through a sequence of positions
    return Commands.sequence(
      turretSubsystem.moveToAngleCommand(0.0),
      Commands.waitSeconds(1.0),
      turretSubsystem.moveToAngleCommand(90.0),
      Commands.waitSeconds(1.0),
      turretSubsystem.moveToAngleCommand(-90.0),
      Commands.waitSeconds(1.0),
      turretSubsystem.moveToAngleCommand(0.0)
    );
  }
}