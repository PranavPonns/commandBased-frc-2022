package Utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class WPIMotor extends com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX implements WMotor {

    public WPIMotor(int deviceNumber) {
        super(deviceNumber);
        //TODO Auto-generated constructor stub
    }

    @Override
    public void setSpeed(double rpm) {
        // TODO Auto-generated method stub

        rpm *= 2048/10;
        this.set(ControlMode.Velocity, rpm);
    }

    @Override
    public void resetEncoder() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getVelcoity() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void setVolts() {
        // TODO Auto-generated method stub
        
    }
    
}
