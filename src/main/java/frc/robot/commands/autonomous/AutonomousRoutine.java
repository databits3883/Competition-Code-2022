// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**Use this as you would a {@link SequentialCommandGroup}
 * except it will automatically be added to the 
 * {@link AutonomousRoutineRegistry} to be selectable
 * by a Shuffleboard user
 */
public abstract class AutonomousRoutine extends SequentialCommandGroup {
  /** Creates a new AutonomousRoutine. */
  public AutonomousRoutine() {
    AutonomousRoutineRegistry.getInstance().register(this);
  }
}
