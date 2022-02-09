// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbSequence extends SequentialCommandGroup {
  private final ClimbArm m_climbArm;
  /** Creates a new ClimbSequence. */
  public ClimbSequence(ClimbArm climbArm) {

    m_climbArm = climbArm;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClimbPull(),
      new DropArmForward(m_climbArm, 0.5, 0.5),
      new ExtendClimbArm(0.5, m_climbArm),
      new RetractIntoBar(m_climbArm)
    );
  }
}
