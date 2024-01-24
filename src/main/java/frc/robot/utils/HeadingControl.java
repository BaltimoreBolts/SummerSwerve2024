package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;


public class HeadingControl {
    
    private Rotation2d m_pastHeading = new Rotation2d();
    private double m_pastTimestamp_s = 0.0;
    private Rotation2d m_goalHeading = new Rotation2d();

    private boolean m_snapEnabled = false;
    
    public void update(double commandedPower, Rotation2d heading) {

        double headingVelocity = (m_pastHeading.getRadians() - heading.getRadians()) / (Timer.getFPGATimestamp() - m_pastTimestamp_s);

        m_pastHeading = heading;
        m_pastTimestamp_s = Timer.getFPGATimestamp();

        boolean previousSnapState = m_snapEnabled;
        m_snapEnabled = !(Math.abs(headingVelocity) < 0.03 && !previousSnapState) && Math.abs(commandedPower) < 0.05;

        if (m_snapEnabled && !previousSnapState) {
            m_goalHeading = heading;
        }
    }

    public Rotation2d getSnapHeading() {
        return m_goalHeading;
    }

    public boolean getIfSnapping() {
        return m_snapEnabled;
    }

}