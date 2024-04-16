package frc.robot.commands.climber;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftClimberSubsystem;
import frc.robot.subsystems.RightClimberSubsystem;


public class SetLeftClimber extends Command {
    private final LeftClimberSubsystem climber;
    private final XboxController controller;

    public SetLeftClimber(LeftClimberSubsystem m_leftClimber, XboxController con){
        climber = m_leftClimber;
        controller = con;
        addRequirements(m_leftClimber);
    }


    @Override
    public void execute(){
        double speed = controller.getLeftY();
        climber.setLeftClimberDown(speed);
    }


    @Override
    public void end(boolean interrupted) {
        // Stop shooter motors when command ends
        climber.stopLeft();
    }
}
