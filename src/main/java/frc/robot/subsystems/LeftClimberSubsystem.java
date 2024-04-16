// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LeftClimberSubsystem extends SubsystemBase {

  private final com.ctre.phoenix6.hardware.TalonFX  leftClimberMotor;

  /** Creates a new LeftClimberSubsystem. */
  public LeftClimberSubsystem() {

         leftClimberMotor = new TalonFX(18);
  }

      public void setLeftClimberDown(double speed){
        leftClimberMotor.set(speed*-1);
    }

      public void setLeftClimberUp(double speed){
        leftClimberMotor.set(speed);
    }

      public void stopLeft() {
        leftClimberMotor.set(0);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
