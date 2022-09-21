// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.StagingConstants.*;

public class CargoStaging extends SubsystemBase {

  private final CANSparkMax m_motor;
  private final DigitalInput m_inSensor = new DigitalInput(SENSOR_CHANNEL);



  StagingState m_currentState;

  StagingState holding;
  StagingState waiting;
  StagingState clearing;
  StagingState running;
  StagingState frozen;

  private Notifier m_autoRunner;
  private double m_lastSet =0;

  /** Creates a new CargoStaging. */
  public CargoStaging() {
    m_motor  = new CANSparkMax(MOTOR_CHANNEL, MotorType.kBrushless);
    m_motor.setInverted(true);
    m_motor.setIdleMode(IdleMode.kBrake);

    


    frozen = new StagingState(){
      public void enterState(){
        autoStop();
        Intake.IntakeDisableFlag = true;
      }
    };
    holding = new StagingState(){
      public void enterState(){
        autoStop();
      }
      public StagingState nextState(){
        if(cargoAtEntrance()) return frozen;
        else return this;
      }
    };
    waiting = new StagingState(){
      public void enterState(){
        autoStop();
      }
      public StagingState nextState(){
        if(cargoAtEntrance()){
          return clearing;
        }
        else return this;
      }
    };
    clearing = new StagingState(){
      public void enterState(){
        autoRunIn();
      }
      public StagingState nextState(){
        if(!cargoAtEntrance()) return running;
        else return this;
      }
    };
    running = new StagingState(){
      Timer m_runintTimer = new Timer();
      public void enterState(){
        m_runintTimer.reset();
        m_runintTimer.start();
      }
      public StagingState nextState(){
        if(m_runintTimer.hasElapsed(Constants.StagingConstants.RUN_TIME)){
          return holding;
        }
        else return this;
      }
    };

    m_currentState = waiting;
    m_autoRunner = new Notifier(()->{
      StagingState nextState = m_currentState.nextState();
      if(nextState != m_currentState){
        m_currentState = nextState;
        m_currentState.enterState();
      }
    });

    resetAutoStaging();

  }

  private void autoRunIn(){
    m_motor.set(IN_SPEED);
    m_lastSet = IN_SPEED;
  }
  private void autoRunOut(){
    m_motor.set(OUT_SPEED);
    m_lastSet = OUT_SPEED;
  }
  private void autoStop(){
    m_motor.set(0);
    m_motor.setInverted(true);
    m_lastSet =0;
  }

  public void runIn(){
    leaveAutoStage();
    if(m_lastSet != IN_SPEED){
      m_motor.set(IN_SPEED);
      m_lastSet = IN_SPEED;
    }
  }
  public void runOut(){
    leaveAutoStage();
    if(m_lastSet != OUT_SPEED){
      m_motor.set(OUT_SPEED);
      m_lastSet = OUT_SPEED;
    }
  }
  public void stop(){
    leaveAutoStage();
    if(m_lastSet !=0){
      m_motor.set(0);
      m_lastSet =0;
    }
    
  }

  public void leaveAutoStage(){
    m_currentState = holding;
    m_autoRunner.stop();
  }
  public boolean resetAutoStaging(){
    m_currentState = waiting;
    m_autoRunner.startPeriodic(0.005);
    return true;
  }

  public boolean cargoAtEntrance(){
    return !m_inSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  abstract class StagingState{
    void enterState(){
      return;
    };
    StagingState nextState(){
      return this;
    }
  }
}