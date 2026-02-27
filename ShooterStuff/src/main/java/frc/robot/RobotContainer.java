// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.RPM;

import java.io.File;
import java.util.Optional;

import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // ========== FIELD CONSTANTS ==========
  private static final double FIELD_LENGTH_M = Units.inchesToMeters(651.25);

  public static final Translation2d HUB_CENTER_BLUE =
      new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));

  public static final Translation2d HUB_CENTER_RED =
      new Translation2d(FIELD_LENGTH_M - Units.inchesToMeters(182.11),
                        Units.inchesToMeters(158.84));

  // ========== CONTROLLERS ==========
  final CommandJoystick joystickL   = new CommandJoystick(0);
  final CommandJoystick joystickR   = new CommandJoystick(1);
  double xV = 0;
  double yV = 0;
  double rV = 0;
  final CommandXboxController driverXbox   = new CommandXboxController(2);
  final CommandXboxController operatorXbox = new CommandXboxController(3);

  // ========== SUBSYSTEMS ==========



  private final HopperSubsystem  hopper  = new HopperSubsystem();
  private final FeederSubsystem  feeder  = new FeederSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  // Vision auto-runs via its periodic() — no manual calls needed anywhere.
  //private final Vision vision = new Vision(drivebase);

  // ========== TURRET CONFIG ==========
  private static final double TURRET_MANUAL_MAX_SPEED_DEG_PER_SEC = 90.0;

  // ========== FEED SYSTEM SPEEDS ==========
  private static final double HOPPER_SPEED = 0.5;
  private static final double FEEDER_SPEED = 0.5;

  // =========================================================================
  //  SHOOT COMMAND FACTORY
  //
  //  Rules:
  //    - Hopper and feeder MUST always run together with the shooter.
  //      Running hopper/feeder without the shooter will jam balls at
  //      the shooter wheel since there's nothing to clear them out.
  //    - Shooter CAN run alone (spin-up / warm-up before feeding).
  //    - The ONLY way to run hopper/feeder is through shootWithFeedCommand().
  // =========================================================================

  /**
   * Full shoot command: runs shooter, hopper, and feeder all at the same time.
   * This is the ONLY command that runs the hopper and feeder — they are never
   * started independently because balls would jam at a stationary shooter wheel.
   *
   * When toggled off, all three stop together automatically.
   */
  private Command shootWithFeedCommand() {
    return shooter.setVelocity(RPM.of(500))
                  .alongWith(
                      hopper.runMotorCommand(HOPPER_SPEED),
                      feeder.runMotorCommand(FEEDER_SPEED)
                  )
                  .withName("ShootWithFeed");
  }

  // ========== AUTO ==========
  private SendableChooser<Command> autoChooser;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);



  }




  private void configureBindings()
  {

    // ── Simulation bindings ───────────────────────────────────────────────
    if (Robot.isSimulation())
    {

    }

    // ── Test mode bindings ────────────────────────────────────────────────
    if (DriverStation.isTest())
    {

      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    }
    else
    {
      // ── Driver bindings ──────────────────────────────────────────────────


      // ── Operator: Turret bindings ─────────────────────────────────────────
     // operatorXbox.a().onTrue(turret.setAngleCommand(0.0));
     // operatorXbox.b().onTrue(turret.setAngleCommand(90.0));
     // operatorXbox.x().onTrue(turret.setAngleCommand(-90.0));
     // operatorXbox.y().onTrue(turret.setAngleCommand(180.0));

      // Left bumper: hard stop turret

      // Right bumper TOGGLE: hub-lock the turret


      // ── Operator: Shoot binding ───────────────────────────────────────────
      //
      // Left trigger (>50%) TOGGLE — full shoot mode.
      //
      // Runs shooter + hopper + feeder simultaneously.
      // This is the ONLY way to run the hopper and feeder — they are never
      // triggered independently because a ball would jam against a stopped
      // shooter wheel. When toggled off, all three stop together.
      operatorXbox.leftTrigger(0.5).toggleOnTrue(shootWithFeedCommand());
    }
  }


  /**
   * Returns the correct hub Translation2d for the current alliance.
   * Falls back to blue if the Driver Station hasn't reported an alliance yet.
   */
  private Translation2d getHubTarget()
  {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return HUB_CENTER_RED;
    }
    return HUB_CENTER_BLUE;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

}