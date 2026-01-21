package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KerklunkSubsystem;

public class LinearActuatorCommand extends Command{
    private final KerklunkSubsystem linearActuator;
    private final double length;

    public LinearActuatorCommand(KerklunkSubsystem subsystem, double length) {
       linearActuator = subsystem;
        this.length = length;
        addRequirements(linearActuator);

    }
    @Override
    public void initialize(){
        linearActuator.setLength(length); //sets the target angle
    }

    @Override
    public void execute(){
        linearActuator.setLength(length); //goes to the target angle
    }

    @Override 
    public void end(boolean interrupted) {
        //kerklunk.zeroAngle(); 
        // enable if servo needs to go to zero when ending; default probably is no
    }

    @Override
    public boolean isFinished()
    {
        return true; //checks to see if it went to the angle
    }
    
}