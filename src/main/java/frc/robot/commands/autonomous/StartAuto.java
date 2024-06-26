// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoShooter;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartAuto extends SequentialCommandGroup {
  /** Creates a new StartAuto. */
  public StartAuto(IntakeSubsystem intake, ShooterSubsystem shooter) {

    addCommands(new AutoShooter(shooter, intake).withTimeout(3));
    
  }


}
