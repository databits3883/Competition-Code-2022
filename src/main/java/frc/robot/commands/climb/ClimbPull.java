// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbArm;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbPull extends CommandBase {
  final ClimbArm m_arm;

  State m_currentState;
  /** Creates a new ClimbPull. */
  public ClimbPull(ClimbArm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm=arm;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setAngleWinchSpeed(WINCH_DESLACK_SPEED);
    m_currentState = State.kFast;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    transitionState();
    m_arm.setExtensionSpeed(m_currentState.motorSpeed);
  }

  void transitionState(){
    switch(m_currentState){
      case kFast:
        if(m_arm.measureArmLength() > LIFT_SLOW_HEIGHT){
          m_currentState = State.kSlow;
        }
        break;
      case kSlow:
        if(m_arm.measureArmLength() > ON_BAR_HEIGHT){
          m_currentState = State.kStop;
        }
        break;
      case kStop:
        m_currentState = State.kStop;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setExtensionSpeed(0.0);
    m_arm.setAngleWinchSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_currentState==State.kStop;
  }

  static enum State{
    kFast(ARM_PULL_FAST_SPEED), 
    kSlow(ARM_PULL_SLOW_SPEED), 
    kStop(0.0);

    double motorSpeed;
    State(double speed){
      motorSpeed = speed;
    }
  }
}
