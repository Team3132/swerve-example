package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Xbox implements Controls {

    private final XboxController m_controller = new XboxController(2);

    @Override
    public double getXSpeed() {
        return m_controller.getLeftY();
    }

    @Override
    public double getYSpeed() {
        return m_controller.getLeftX();
    }

    @Override
    public double getTurn() {
        return m_controller.getRightX();
    }

}
