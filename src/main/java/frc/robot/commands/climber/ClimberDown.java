package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDown extends Command {
    private final ClimberSubsystem climber;

    public ClimberDown(ClimberSubsystem climberSubsystem){
        climber = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute(){
        climber.moveDown(1);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop shooter motors when command ends
        climber.stop();
    }
}
