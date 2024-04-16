// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoShooter;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveBackwards;
import frc.robot.commands.MoveForward;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartAutoTwo extends SequentialCommandGroup {
  /** Creates a new StartAuto. */
  public StartAutoTwo(IntakeSubsystem intake, ShooterSubsystem shooter, DriveTrain drivetrain) {

    addCommands(
      new AutoShooter(shooter,intake).withTimeout(0.5),
      new ParallelCommandGroup(new IntakeCommand(intake), new MoveForward(drivetrain)).withTimeout(3.5),
      new MoveBackwards(drivetrain).withTimeout(3.5),
      new AutoShooter(shooter, intake).withTimeout(0.5)
      )
      
      ;
    
  }


}
