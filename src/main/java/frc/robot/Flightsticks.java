package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Flightsticks implements Controls {

    private final Joystick m_left = new Joystick(0);
    private final Joystick m_right = new Joystick(1);

    @Override
    public double getXSpeed() {
        return m_left.getY();
    }

    @Override
    public double getYSpeed() {
        return m_left.getX();
    }

    @Override
    public double getTurn() {
        return m_right.getX();
    }

}
