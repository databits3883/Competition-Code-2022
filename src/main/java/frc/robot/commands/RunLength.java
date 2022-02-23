// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimbArm;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunLength extends CommandBase {
  ClimbArm m_climb;
  Double m_lengthSpeed = 0.0;
  /** Creates a new ExtendIntake. */
  public RunLength(ClimbArm climb, Double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_climb = climb;
    m_lengthSpeed = speed;
    addRequirements(climb);
  }

  @Override
  public void initialize(){
    m_climb.setExtensionSpeed(m_lengthSpeed);
  }

  @Override
  public boolean isFinished(){
    boolean checkIfFinished;
    double currentLength = m_climb.measureArmLength();
    if (m_lengthSpeed > 0.0) { 
        checkIfFinished = (currentLength > 20.0);
    } else {
        checkIfFinished = (currentLength < 5.0);
    }
    checkIfFinished = false;
    return checkIfFinished;
  }
  @Override
  public void end(boolean interrupted) {
    m_climb.setExtensionSpeed(0);
  }
}
