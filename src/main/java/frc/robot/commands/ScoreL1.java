// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreL1 extends Command {
  private ElevatorSubsystem m_elevator;
  private IntakeSubsystem m_intake;

  /** Creates a new ScoreL4. */
  public ScoreL1() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = new IntakeSubsystem();
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {
      m_intake.setPivotSpeed(.05);
      wait(750);
      m_intake.stopPivot();
      m_intake.intakeCoral(.8);
    } catch (InterruptedException e) {
      m_intake.stopPivot();
      m_intake.intakeCoral(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
