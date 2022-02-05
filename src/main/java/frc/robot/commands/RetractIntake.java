// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.IntakeConstants.RETRACT_TIMEOUT;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractIntake extends ParallelRaceGroup {
  /** Creates a new RetractIntake. */
  public RetractIntake(Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(RETRACT_TIMEOUT),
      new FunctionalCommand(intake::startRetract,
       ()->{return;},
       (interrupted)->{
         if(interrupted){
           intake.disableRetractMotor();
         }
       },
       intake::getRetracted, 
       intake)
    );
  }
}
