

package frc.robot.commands.climber;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.RightClimberSubsystem;


public class SetRightClimber extends Command {
    private final RightClimberSubsystem climber;
    private final XboxController controller;

    public SetRightClimber(RightClimberSubsystem climberSubsystem, XboxController con){
        climber = climberSubsystem;
        controller = con;
        addRequirements(climberSubsystem);
    }


    @Override
    public void execute(){
        double speed = controller.getRightY();
        climber.setRightClimberDown(speed);
    }


    @Override
    public void end(boolean interrupted) {


        climber.stopRight();
    }
}
