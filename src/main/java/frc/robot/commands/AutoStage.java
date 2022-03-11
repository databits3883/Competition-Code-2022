package frc.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import frc.robot.subsystems.CargoStaging;

import static frc.robot.Constants.StagingConstants.*;
public class AutoStage extends NotifierCommand{
    
    CargoStaging m_staging;

    public AutoStage(CargoStaging mechanism){
        super((new StagingStateMachine(mechanism)), 0.001, mechanism);
        m_staging = mechanism;

        addRequirements(mechanism);
    }

    @Override
    public void initialize(){
        super.initialize();
        m_staging.stop();
    }

    
    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        m_staging.stop();
    }


}

class StagingStateMachine implements Runnable{
    enum StagingState{waiting,clearing,runningExtra};

    CargoStaging m_mechanism;
        StagingState m_state;
        Timer m_timer = new Timer();

        public StagingStateMachine(CargoStaging mechanism){
            m_state = StagingState.waiting;
            m_mechanism = mechanism;
        }

        public void run(){
            switch (m_state){
                case runningExtra:
                    if(m_timer.hasElapsed(RUN_TIME)){
                        m_state = StagingState.waiting;
                        m_mechanism.stop();
                        m_timer.stop();
                    }
                    break;
                case clearing:
                    if(!m_mechanism.cargoAtEntrance()){
                        m_state = StagingState.runningExtra;
                        m_timer.reset();
                        m_timer.start();
                    }
                    break;
                case waiting:
                    if(m_mechanism.cargoAtEntrance()){
                        m_state=StagingState.clearing;
                        m_mechanism.runIn();
                        
                    }
                    break;
            }
        }
}

