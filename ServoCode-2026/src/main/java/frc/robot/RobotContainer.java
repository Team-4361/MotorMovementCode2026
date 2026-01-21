// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.KerklunkCommand;
import frc.robot.commands.LinearActuatorCommand;
import frc.robot.subsystems.KerklunkSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  final CommandXboxController driverXbox = new CommandXboxController(0);
  public static KerklunkSubsystem Longestkerklunk = new KerklunkSubsystem(3);
  public static KerklunkSubsystem Longerkerklunk = new KerklunkSubsystem(2);
  public static KerklunkSubsystem Shorterkerklunk = new KerklunkSubsystem(1);
  public static KerklunkSubsystem Shortestkerklunk = new KerklunkSubsystem(0);




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
        //.onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
    //goes 1 5/8" in + 10/8 in from start
     // longest starting shaft length 3/8 in ~0.9 cm
    driverXbox.a().onTrue(new LinearActuatorCommand(Shortestkerklunk, 0.2));
    driverXbox.rightBumper().onTrue(new LinearActuatorCommand(Shortestkerklunk, 0.30));
    driverXbox.x().onTrue(new LinearActuatorCommand(Shortestkerklunk, 0.40));
   //2 1/4"
    driverXbox.y().onTrue(new LinearActuatorCommand(Shortestkerklunk, 0.60));
    //10.7 cm
    driverXbox.b().onTrue(new LinearActuatorCommand(Shortestkerklunk, 1));
    //full extentsion 14.7 cm / 5 1/2" in
    //driverXbox.b().onTrue(new KerklunkCommand(Longestkerklunk, 180.0));
    //driverXbox.y().onTrue(new LinearActuatorCommand(Longestkerklunk, 90.0));

  
    

    
    



  }

}
