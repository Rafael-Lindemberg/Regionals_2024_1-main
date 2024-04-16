package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RightClimberSubsystem extends SubsystemBase{

    private final com.ctre.phoenix6.hardware.TalonFX  rightClimberMotor;

       public RightClimberSubsystem() {

         rightClimberMotor = new TalonFX(17);
    }

    public void setRightClimberDown(double speed) {   
        rightClimberMotor.set(speed*-1);
    }

    
    public void setRightClimberUp(double speed) {   
        rightClimberMotor.set(speed);
    }


    public void stopRight() {
        rightClimberMotor.set(0);
    }

}
