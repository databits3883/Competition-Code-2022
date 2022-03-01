// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIntakeRunning extends InstantCommand {

  private final Intake m_intake;
  private final int m_direction;


  public SetIntakeRunning(Intake intake, int direction) {
    m_intake = intake;
    m_direction = direction;

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.takeInOrOut(Constants.IntakeConstants.spinSpeed * m_direction);
  }
}
