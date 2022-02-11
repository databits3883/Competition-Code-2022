// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.Constants.DriveConstants.*;

import java.util.concurrent.atomic.AtomicBoolean;
public class Drivetrain extends SubsystemBase {

  boolean m_allCalibrated = false;
  boolean m_currentlyCalibrating = false;

  final SwerveDriveKinematics m_kinematics;
  final Module[] m_modules = new Module[4];
  final AHRS m_gyro;


  final SwerveDriveOdometry m_odometry;
  SwerveModuleState[] m_lastMeasuredStates = new SwerveModuleState[4];

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_kinematics = new SwerveDriveKinematics(
      new Translation2d(DRIVE_TRACK_LENGTH/2, DRIVE_TRACK_WIDTH/2), //fron right 
      new Translation2d(-DRIVE_TRACK_LENGTH/2, DRIVE_TRACK_WIDTH/2), //rear right
      new Translation2d(-DRIVE_TRACK_LENGTH/2, -DRIVE_TRACK_WIDTH/2), //rear left
      new Translation2d(DRIVE_TRACK_LENGTH/2, -DRIVE_TRACK_WIDTH/2) //front left
    );
    
    m_modules[0] = new Module(CANChannels.FRONT_RIGHT_VELOCITY, CANChannels.FRONT_RIGHT_ROTATION, CalibrationConstants.FRONT_RIGHT_SWITCH_LOCATION);
    m_modules[1] = new Module(CANChannels.REAR_RIGHT_VELOCITY, CANChannels.REAR_RIGHT_ROTATION, CalibrationConstants.REAR_RIGHT_SWITCH_LOCATION);
    m_modules[2] = new Module(CANChannels.REAR_LEFT_VELOCITY, CANChannels.REAR_LEFT_ROTATION, CalibrationConstants.REAR_LEFT_SWITCH_LOCATION);
    m_modules[3] = new Module(CANChannels.FRONT_LEFT_VELOCITY, CANChannels.FRONT_LEFT_ROTATION, CalibrationConstants.FRONT_LEFT_SWITCH_LOCATION);

    m_gyro = new AHRS(I2C.Port.kMXP,(byte)200);

    m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());
  }

  public void setSpeedFieldRelative(ChassisSpeeds speeds){
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
     speeds.omegaRadiansPerSecond, Rotation2d.fromDegrees(m_gyro.getYaw()));
     setChassisSpeed(speeds);
  }

  public void setChassisSpeed(ChassisSpeeds speeds){
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    setStates(states);
  }
  private void setStates(SwerveModuleState[] states){
    if(!m_currentlyCalibrating){
      SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_WHEEL_SPEED);
      for(int i=0;i<4;i++){
        m_modules[i].setState(states[i]);
      }
    }
  }

  public boolean getAllCalibrated(){
    return m_allCalibrated;
  }

  public void startCalibration(){
    m_gyro.calibrate();
    m_currentlyCalibrating = true;
    for(Module m : m_modules){
      m.startCalibration();
    }
  }
  public void resetGyro(){
    m_gyro.reset();
    m_odometry.resetPosition(new Pose2d(), m_gyro.getRotation2d());
  }

  void measureCurrentStates(){
    for(int i=0;i<4;i++){
      m_lastMeasuredStates[i] = m_modules[i].measureState();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measureCurrentStates();
    m_odometry.update(m_gyro.getRotation2d(), m_lastMeasuredStates);

    if(m_currentlyCalibrating){
      boolean doneCalibrating = true;
      for(Module m : m_modules){
        doneCalibrating &= m.isCalibrated();
      }
      if(doneCalibrating){
        m_allCalibrated = true;
        m_currentlyCalibrating = false;
      }
    }
  }

  //TODO:
  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("X location", ()->m_odometry.getPoseMeters().getX(), null);
    builder.addDoubleProperty("Y location", ()->m_odometry.getPoseMeters().getY(), null);
  }

  private class Module{
    private CANSparkMax m_rotationMotor;
    private RelativeEncoder m_rotationEncoder;
    private SparkMaxPIDController m_rotationController;

    private CANSparkMax m_velocityMotor;
    private RelativeEncoder m_velocityEncoder;
    private SparkMaxPIDController m_velocityController;

    private SparkMaxLimitSwitch m_calibrationSwitch;

    private Calibrator m_calibrator;
    private Thread m_calibratorThread;

    private boolean m_calibrating = false;

    Module(int velocityChannel, int rotationChannel, double calibrationSwitchLocation){
      m_rotationMotor = new CANSparkMax(rotationChannel, MotorType.kBrushless);
      m_rotationEncoder = m_rotationMotor.getAlternateEncoder(4096);
      m_rotationController = m_rotationMotor.getPIDController();

      m_rotationMotor.setInverted(true);
      m_rotationEncoder.setInverted(true);
      m_rotationEncoder.setPositionConversionFactor(ROTATION_GEARING * Math.PI*2);
      m_rotationController.setFeedbackDevice(m_rotationEncoder);

      m_rotationController.setP(0.6);
      m_rotationController.setI(0);
      m_rotationController.setD(0);
      m_rotationController.setFF(0);

      m_velocityMotor = new CANSparkMax(velocityChannel, MotorType.kBrushless);
      m_velocityEncoder = m_velocityMotor.getEncoder();
      m_velocityController = m_velocityMotor.getPIDController();

      m_velocityEncoder.setVelocityConversionFactor(VELOCITY_GEARING*WHEEL_CIRCUMFRENCE * (1.0/60.0));

      m_velocityController.setP(0.22);
      m_velocityController.setI(0);
      m_velocityController.setD(1.2);
      m_velocityController.setFF(0.23);

      m_calibrationSwitch = m_velocityMotor.getReverseLimitSwitch(Type.kNormallyOpen);
      m_calibrator = new Calibrator(m_rotationMotor, m_rotationEncoder, m_calibrationSwitch, calibrationSwitchLocation);
      m_calibratorThread = new Thread(m_calibrator);


      //TODO: remove
      Shuffleboard.getTab("Tab 5").addNumber("Rotation encoder "+m_rotationMotor.getDeviceId(), m_rotationEncoder::getPosition);
    }

    public SwerveModuleState measureState(){
      return new SwerveModuleState(m_velocityEncoder.getVelocity(),new Rotation2d(m_rotationEncoder.getPosition()));
    }

    public void setState(SwerveModuleState state){
      state = SwerveModuleState.optimize(state, new Rotation2d(m_rotationEncoder.getPosition()));
      m_velocityController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
      m_rotationController.setReference(
        mapAngleToNearContinuous(state.angle.getRadians()),
        ControlType.kPosition);
    }

    double mapAngleToNearContinuous(double newAngle){
      double currentAngle = m_rotationEncoder.getPosition();
      long completedRotations = Math.round(currentAngle / (Math.PI*2));
      double offsetAngle = newAngle%(Math.PI*2) + (2*Math.PI*completedRotations);
      if(Math.abs(currentAngle - offsetAngle) < Math.PI){
          return offsetAngle;
      }else if(offsetAngle > currentAngle){
          return offsetAngle - Math.PI*2;
      }else{
          return offsetAngle + Math.PI*2;
      }
  }


    public void startCalibration(){
      if(!m_calibrator.m_calibrated){
        m_calibratorThread.start();
        m_calibrating = true;
      }

    }

    public boolean isCalibrated(){
      if(m_calibrating){
        if(m_calibrator.m_calibrated){
          m_calibrating = false;
          return true;
        }else{
          return false;
        }
      }else{
        return m_calibrator.m_calibrated;
      }
    }


    

  }
  class Calibrator implements Runnable{
    private SparkMaxLimitSwitch m_calibrationSwitch;
    private CANSparkMax m_motor;
    private RelativeEncoder m_encoder;
    private double m_trueSwitchLocation;

    volatile boolean m_calibrated;
    private double m_riseLocation;
    private double m_fallLocation;

    volatile boolean m_allowCalibration ;

    public Calibrator(CANSparkMax motor, RelativeEncoder encoder,SparkMaxLimitSwitch calibrationSwitch, double switchLocation){
      m_motor = motor;
      m_encoder = encoder;
      m_calibrationSwitch = calibrationSwitch;
      m_trueSwitchLocation = switchLocation;

      m_calibrated = false;
      m_allowCalibration = true;
    }


    public void run() {
      if(m_allowCalibration){
        m_motor.set(CalibrationConstants.CALIBRATION_SPEED);

        boolean riseCalibrated = false;
        boolean fallCalibrated = false;

        boolean lastSwitchValue = m_calibrationSwitch.isPressed();
        boolean currentSwitchValue;
        while(!(riseCalibrated && fallCalibrated) && m_allowCalibration){
          currentSwitchValue = m_calibrationSwitch.isPressed();
          if(!riseCalibrated){
            if(currentSwitchValue & !lastSwitchValue){
              m_riseLocation = m_encoder.getPosition();
              riseCalibrated = true;
            }
          }else if(!fallCalibrated){
            if(lastSwitchValue & !currentSwitchValue){
              m_fallLocation = m_encoder.getPosition();
              fallCalibrated =true;
            }
          }

          lastSwitchValue = currentSwitchValue;
          try{
            Thread.sleep(CalibrationConstants.CALIBRATION_WAIT_MILLIS,CalibrationConstants.CALIBRATION_WAIT_NANOS);
          }catch(InterruptedException e){
            System.err.println("Calibration wait interrupted:\n"+e.getMessage());
            m_motor.set(0);
            return;
          }
        }
        m_motor.set(0);
          double switchCenter = (m_fallLocation+m_riseLocation)/2;
          double stopLocation = m_encoder.getPosition() - switchCenter;
          m_encoder.setPosition((new Rotation2d(m_trueSwitchLocation+stopLocation)).getRadians());

          m_calibrated = true;
          m_allowCalibration = false;
        
      }
    }

  }
}
