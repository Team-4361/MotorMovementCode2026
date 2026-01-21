package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KerklunkSubsystem;

public class KerklunkCommand extends Command{
    private final KerklunkSubsystem kerklunk;
    private final double targetAngle;

    public KerklunkCommand(KerklunkSubsystem subsystem, double targetAngle) {
        this.kerklunk = subsystem;
        this.targetAngle = targetAngle;
        addRequirements(kerklunk);

    }
    @Override
    public void initialize(){
        kerklunk.setAngle(targetAngle); //sets the target angle
    }

    @Override
    public void execute(){
        kerklunk.setAngle(targetAngle); //goes to the target angle
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