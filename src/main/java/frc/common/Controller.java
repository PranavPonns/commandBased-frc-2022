package frc.common;

import edu.wpi.first.wpilibj.Joystick;

public class Controller {
    private final Joystick controller;
    
    public Controller(int portID){
        controller = new Joystick(portID);
    }    

    public double getRawX(){
        return controller.getX();
    }

    public double getRawY(){
        return controller.getZ();
    }

    public double getRawZ(){
        return controller.getZ();
    }
    
    
}
