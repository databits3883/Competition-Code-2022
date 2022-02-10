// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
  private final CANSparkMax takeMotor = new CANSparkMax(INTAKE_CHANNEL,MotorType.kBrushed);
  
  private final MotorController raiseAndLowerMotor = new CANSparkMax(RAISE_LOWER_CHANNEL,MotorType.kBrushed);
  private final DigitalInput extendLimit = new DigitalInput(EXTEND_LIMIT_CHANNEL);
  private final DigitalInput retractLimit = new DigitalInput(RETRACT_LIMIT_CHANNEL);
  private final AsynchronousInterrupt retractStopInterrupt;
  private final AsynchronousInterrupt extendStopInterrupt; 
  private boolean isRetracted = true;
  private boolean isExtended = false;







  
  public Intake() {
    takeMotor.setIdleMode(IdleMode.kBrake);
    retractStopInterrupt = new AsynchronousInterrupt(retractLimit, this::stopRetract);
    extendStopInterrupt = new AsynchronousInterrupt(extendLimit, this::stopExtend);
  }
  
  public void takeInOurOut(double speed){
    takeMotor.set(speed);
  }

  public void startExtend(){
    raiseAndLowerMotor.set(extendSpeed);
    isExtended = false;
    isRetracted = false;
    extendStopInterrupt.enable();
  }
  public void startRetract(){
    raiseAndLowerMotor.set(retractSpeed);
    isExtended = false;
    isRetracted = false;
    retractStopInterrupt.enable();
  }
  

  public void disableExtendMotor(){

    retractStopInterrupt.disable();
    extendStopInterrupt.disable();
    raiseAndLowerMotor.set(0.0);
  }

  public void disableRetractMotor(){

    retractStopInterrupt.disable();
    extendStopInterrupt.disable();
    raiseAndLowerMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getExtended(){
    return isExtended;
  }

  public boolean getRetracted(){
    return isRetracted;
  }


  public void stopExtend(boolean risingEdge, boolean fallingEdge){
    raiseAndLowerMotor.set(0);
    isExtended = true;
    isRetracted = false;
    extendStopInterrupt.disable();

  }
  public void stopRetract(boolean risingEdge, boolean fallingEdge){
    raiseAndLowerMotor.set(0);
    isExtended = false;
    isRetracted = true;
    retractStopInterrupt.disable();
  }
  


}
