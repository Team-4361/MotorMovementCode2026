package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final CommandXboxController driver = new CommandXboxController(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        driver.rightBumper().whileTrue(shooter.set(0.1));

        driver.start().onTrue(Commands.runOnce(SignalLogger::start));
        driver.back().onTrue(Commands.runOnce(SignalLogger::stop));

        driver.a().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driver.b().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        driver.x().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driver.y().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public edu.wpi.first.wpilibj2.command.Command getAutonomousCommand() {
        return null;
    }
}