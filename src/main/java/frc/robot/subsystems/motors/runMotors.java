package frc.robot.subsystems.motors;

import com.ctre.phoenix6.hardware.TalonFX;

public class runMotors {
    
    private final static int MotorID = 50;
    private TalonFX motor;
    private final static int MotorID2 = 42;
    private TalonFX motor2;
    
    public runMotors() {
        motor = new TalonFX(MotorID, "Collector 2025");
        motor2 = new TalonFX(MotorID2, "Collector 2025");
    }

    public void runMotorsfor(){
        motor.set(1);
        motor2.set(-1);
    }

    public void stopMotors(){
        motor.set(0);
        motor2.set(0);
    }

    public void runMotorsfor2(){
        motor.set(-1);
        motor2.set(1);
    }

}
