package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;

public class DemoJoystick extends Joystick {

    Joystick m_trusted;
    int m_deadButtonStick;
    int m_deadButtonButtons;

    public DemoJoystick(int port, Joystick trusted, int deadButtonStick, int deadButtonButtons) {
        super(port);
        m_trusted = trusted;
        m_deadButtonStick = deadButtonStick;
        m_deadButtonButtons = deadButtonButtons;
    }
    
    @Override
    public double getRawAxis(int axis) {
        // TODO Auto-generated method stub
        if(m_trusted.getRawButton(m_deadButtonButtons)){
            return (m_trusted.getRawAxis(3)+1)/2* super.getRawAxis(axis);
        }else{
            return m_trusted.getRawAxis(axis);
        }
    }
    @Override
    public boolean getRawButton(int button){
        if (m_trusted.getRawButton(m_deadButtonStick)){
            return super.getRawButton(button);
        }
        else{
            return m_trusted.getRawButton(button);
        }
        
    }
}
