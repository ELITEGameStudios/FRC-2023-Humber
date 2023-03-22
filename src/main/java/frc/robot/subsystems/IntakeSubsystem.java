package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Controllers.DriverController;


public class IntakeSubsystem extends SubsystemBase {
    
    private CANSparkMax m_intake_motor;
    private CANSparkMax pivot_intake_motor;
    private double INTAKE_SPEED = 0.5; 
    private double PIVOT_SPEED= 0.5; 


    public IntakeSubsystem(){

        m_intake_motor = new CANSparkMax(RobotMap.LEFT_MOTOR_1, MotorType.kBrushless);
        pivot_intake_motor = new CANSparkMax(RobotMap.LEFT_MOTOR_2, MotorType.kBrushless);
    }

    public void retrieveGamepiece(){
        m_intake_motor.set(INTAKE_SPEED);
    }
    public void releaseGamepiece(){
        m_intake_motor.set(-INTAKE_SPEED);
    }

    public void pivot(double speed){
        pivot_intake_motor.set(speed * PIVOT_SPEED);
    }
}
