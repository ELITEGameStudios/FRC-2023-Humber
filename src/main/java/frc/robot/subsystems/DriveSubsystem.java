package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Controllers.DriverController;



public class DriveSubsystem extends SubsystemBase {

    private CANSparkMax m_leftMotor_1;
    private CANSparkMax m_leftMotor_2;
    private CANSparkMax m_rightMotor_1;
    private CANSparkMax m_rightMotor_2;
    private MotorControllerGroup m_leftMotor;
    private MotorControllerGroup m_rightMotor;
    private DifferentialDrive m_robotDrive;

    public DriveSubsystem(){
        m_leftMotor_1 = new CANSparkMax(RobotMap.LEFT_MOTOR_1, MotorType.kBrushless);
        m_leftMotor_2 = new CANSparkMax(RobotMap.LEFT_MOTOR_2, MotorType.kBrushless);
    
        m_rightMotor_1 = new CANSparkMax(RobotMap.RIGHT_MOTOR_1, MotorType.kBrushless);
        m_rightMotor_2 = new CANSparkMax(RobotMap.RIGHT_MOTOR_2, MotorType.kBrushless);
    
        m_leftMotor = new MotorControllerGroup(m_leftMotor_1, m_leftMotor_2);
        m_rightMotor = new MotorControllerGroup(m_rightMotor_1, m_rightMotor_2);
    
        m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    }

    public void standardDrive(double speed, double rotation){
        m_robotDrive.arcadeDrive(speed, rotation);
    }
}
