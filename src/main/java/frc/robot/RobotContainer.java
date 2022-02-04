// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RunIntake;
import frc.robot.commands.StageCargo;
import frc.robot.subsystems.CargoStaging;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private SendableChooser<Command> m_autonomousChooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  
  private final Command m_defaultAutonomous = new PrintCommand("No Autonomous Selected");
  private final Joystick m_stick = new Joystick(0);
  private final Intake m_intake = new Intake();
  private final CargoStaging m_staging = new CargoStaging();

  private final JoystickButton m_intakeButton = new JoystickButton(m_stick, 3);
  private final JoystickButton m_outtakeButton = new JoystickButton(m_stick, 2);
  private final JoystickButton m_stageInButton = new JoystickButton(m_stick, 4);
  private final JoystickButton m_stageOutButton = new JoystickButton(m_stick, 5);

  private final RunIntake m_takeIn = new RunIntake(m_intake,1);
  private final RunIntake m_takeOut = new RunIntake(m_intake,-1);
  private final StageCargo m_stageIn = new StageCargo(m_staging, 1);
  private final StageCargo m_stageOut = new StageCargo(m_staging, -1);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutonomousRoutines();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    m_intakeButton.whenHeld(m_takeIn);
    m_outtakeButton.whenHeld(m_takeOut);
    m_stageInButton.whenHeld(m_stageIn);
    m_stageOutButton.whenHeld(m_stageOut);

  }
  /**Configures the autonomous sendable chooser */
  private void configureAutonomousRoutines(){
    m_autonomousChooser.setDefaultOption("NO AUTONOMOUS", m_defaultAutonomous);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autonomousChooser.getSelected();
  }
}
