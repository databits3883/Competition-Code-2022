// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbArm extends SubsystemBase {
  final MotorController m_armLengthWinchMotor;
  final MotorController m_armTiltWinchMotor;
  final Encoder m_armLengthEncoder;
  final Encoder m_armTiltEncoder;

  final ArmFeedforward m_unloadedArmController;
  final PIDController m_armAngleAdjustor;

  final ElevatorFeedforward m_armLenghtController;
  final PIDController m_armLengthAdjustor;



  /** Creates a new ClimbArm. */
  public ClimbArm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
