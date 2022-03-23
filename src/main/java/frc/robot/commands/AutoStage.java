package frc.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import frc.robot.commands.StagingStateMachine.StagingState;
import frc.robot.subsystems.CargoStaging;

import static frc.robot.Constants.StagingConstants.*;
public class AutoStage extends NotifierCommand{
    
    volatile CargoStaging m_staging;
    static volatile StagingState m_state;

    public AutoStage(CargoStaging mechanism){
        super((new StagingStateMachine(mechanism)), 0.001, mechanism);
        m_staging = mechanism;

        addRequirements(mechanism);
    }

    @Override
    public void initialize(){
        super.initialize();
        m_staging.stop();
        m_state = StagingState.waiting;
        
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
    enum StagingState{waiting,clearing,runningExtra,holding};

    volatile CargoStaging m_mechanism;
    
    private Timer m_timer = new Timer();

        public StagingStateMachine(CargoStaging mechanism){
            AutoStage.m_state = StagingState.waiting;
            m_mechanism = mechanism;
        }

        public void run(){
            switch (AutoStage.m_state){
                case runningExtra:
                    if(m_timer.hasElapsed(RUN_TIME)){
                        AutoStage.m_state = StagingState.holding;
                        m_mechanism.stop();
                        m_timer.stop();
                    }
                    break;
                case clearing:
                    if(!m_mechanism.cargoAtEntrance()){
                        AutoStage.m_state = StagingState.runningExtra;
                        m_timer.reset();
                        m_timer.start();
                    }
                    break;
                case waiting:
                    if(m_mechanism.cargoAtEntrance()){
                        AutoStage.m_state=StagingState.clearing;
                        m_mechanism.runIn();
                        
                    }
                    break;
                case holding:
                    
                    

            }
        }
}

