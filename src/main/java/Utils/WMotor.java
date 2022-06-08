package Utils;

public interface WMotor{

    /** 
     * Sets the motor to a certain velocity (m/s)
     * @param speed Speed(m/s) to set motor at
     */
    public void setSpeed(double speed);

    /**
     * Inverts the motor output
     * @param invert true is inverted
     */
    public void setInverted(boolean invert);

    public void resetEncoder();

    public void getVelcoity();

    public void setVolts();

}
