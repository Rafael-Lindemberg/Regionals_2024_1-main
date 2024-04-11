package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

public class ClimberSubsystem extends SubsystemBase {
    private final com.ctre.phoenix6.hardware.TalonFX  climberMotor;

    public ClimberSubsystem() {
         climberMotor = new TalonFX(17);
    }

    public void moveUp(double speed) {   
        climberMotor.set(speed);
    }

    public void moveDown(double speed){
        climberMotor.set(speed*-1);
    }

    public void stop() {
        climberMotor.set(0);
    }

}