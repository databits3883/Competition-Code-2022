// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbArm extends SubsystemBase {
  final MotorController m_armLengthWinchMotor;
  final MotorController m_armAngleWinchMotor;
  final Encoder m_armLengthEncoder;
  final Encoder m_armAngleEncoder;

  final ArmFeedforward m_armAngleController;
  final PIDController m_armAngleAdjustor;

  final ElevatorFeedforward m_armLenghtController;
  final PIDController m_armLengthAdjustor;

  final TrapezoidProfile.Constraints m_armAngleConstraints;
  final TrapezoidProfile.Constraints m_armLengthConstraints;
  

  TrapezoidProfile m_armLengthProfile;
  TrapezoidProfile m_armAngleProfile;

  double armLengthTarget;
  double armAngleTarget;



  /** Creates a new ClimbArm. */
  public ClimbArm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //create final target states
    TrapezoidProfile.State targetAngle = new TrapezoidProfile.State(armAngleTarget,0);
    TrapezoidProfile.State targetLength = new TrapezoidProfile.State(armLengthTarget,0);
    //measure current states
    TrapezoidProfile.State currentAngle = new TrapezoidProfile.State(m_armAngleEncoder.getDistance(),m_armAngleEncoder.getRate());
    TrapezoidProfile.State currentLength = new TrapezoidProfile.State(m_armLengthEncoder.getDistance(),m_armLengthEncoder.getRate());
    //create new profiles based on current state and target
    m_armAngleProfile = new TrapezoidProfile(m_armAngleConstraints,targetAngle,currentAngle);
    m_armLengthProfile = new TrapezoidProfile(m_armLengthConstraints,targetLength,currentLength);
    //calculate current setpoints 
    TrapezoidProfile.State referenceAngle = m_armAngleProfile.calculate(0.02);
    TrapezoidProfile.State referenceLength = m_armLengthProfile.calculate(0.02);
    //calculate new voltage outputs with feedforward and PID correction
    double angleVoltage = m_armAngleController.calculate(referenceAngle.position, referenceAngle.velocity)
      + m_armAngleAdjustor.calculate(currentAngle.position, referenceAngle.position);
    double lengthVoltage = m_armLenghtController.calculate(referenceLength.velocity)
      + m_armLengthAdjustor.calculate(currentLength.position, referenceLength.position);
    //set output voltages
    m_armAngleWinchMotor.setVoltage(angleVoltage);
    m_armLengthWinchMotor.setVoltage(lengthVoltage);
  }
}
