package frc.robot.subsystems;

//import frc.robot.Constants;
//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{

    private final Servo servoArm = new Servo(6);

    private double servoArmPos;

    public Arm(){
        reset();
    }

    public void reset() {
        servoArmPos = 0.5;
    
        servoArm.set(servoArmPos);
    }

    public void incrementTilt(double delta) {
        servoArmPos = saturateLimit(servoArmPos + delta, 0, 1);
        servoArm.set(servoArmPos);
    }

    // Limit motor range to avoid moving beyond safe ranges
  public double saturateLimit(double val, double l_limit, double u_limit) {
    double outval = val;
    if(val > u_limit) {
      outval =  u_limit;
    } else if (val < l_limit) {
      outval = l_limit;
    }
    return outval;
  }





}
