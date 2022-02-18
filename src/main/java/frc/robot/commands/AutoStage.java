package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoStaging;

import static frc.robot.Constants.StagingConstants.*;
public class AutoStage extends CommandBase{
    private final CargoStaging m_mechanism;
    private Timer m_timer = new Timer();

    private StagingState m_state;

    public AutoStage(CargoStaging mechanism){
        m_mechanism = mechanism;

        addRequirements(mechanism);
    }

    @Override
    public void initialize(){
        m_state = StagingState.waiting;
        m_mechanism.stop();
    }

    @Override
    public void execute(){
        switch (m_state){
            case running:
                if(m_timer.hasElapsed(RUN_TIME)){
                    m_state = StagingState.waiting;
                    m_mechanism.stop();
                    m_timer.stop();
                }
                break;
            case waiting:
                if(m_mechanism.cargoAtEntrance()){
                    m_state=StagingState.running;
                    m_mechanism.runIn();
                    m_timer.reset();
                    m_timer.start();
                }
                break;
        }
    }
    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        m_mechanism.stop();
    }


    enum StagingState{waiting,running};
}
