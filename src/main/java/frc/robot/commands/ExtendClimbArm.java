// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbArm;

public class ExtendClimbArm extends CommandBase {

  private double m_distanceToExtend;
  private ClimbArm m_arms;
  private double m_startDistance;

  /** Creates a new ExtendClimbArm. */
  public ExtendClimbArm(double distance, ClimbArm arms) {

    m_distanceToExtend = distance;
    m_arms = arms;
    m_startDistance = m_arms.measureArmLength();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arms);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arms.setExtensionSpeed(Constants.ClimbConstants.CLIMB_ARM_EXTEND_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arms.setExtensionSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_arms.measureArmLength() < m_distanceToExtend + m_startDistance;
  }
}
