// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controllers.ControllerMap;
import frc.robot.Controllers.DriverController;
import frc.robot.Controllers.OperatorController;

public class Robot extends TimedRobot {
  private final CANSparkMax m_leftMotor_1 = new CANSparkMax(RobotMap.LEFT_MOTOR_1, MotorType.kBrushless);
  private final CANSparkMax m_leftMotor_2 = new CANSparkMax(RobotMap.LEFT_MOTOR_2, MotorType.kBrushless);

  private final CANSparkMax m_rightMotor_1 = new CANSparkMax(RobotMap.RIGHT_MOTOR_1, MotorType.kBrushless);
  private final CANSparkMax m_rightMotor_2 = new CANSparkMax(RobotMap.RIGHT_MOTOR_2, MotorType.kBrushless);


  private final CANSparkMax e_motor_1 = new CANSparkMax(RobotMap.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax e_motor_2 = new CANSparkMax(RobotMap.ELEVATOR_MOTOR_ID_2, MotorType.kBrushless);

  private final CANSparkMax elbow = new CANSparkMax(RobotMap.ELBOW_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax shoulder = new CANSparkMax(RobotMap.SHOULDER_MOTOR_ID, MotorType.kBrushless);

  //private final DigitalInput  e_top_limit = new DigitalInput(RobotMap.LIMIT_SWITCH_ID_1);
  //private final DigitalInput  e_bottom_limit = new DigitalInput(RobotMap.LIMIT_SWITCH_ID_2);

  private final MotorControllerGroup m_leftMotor = new MotorControllerGroup(m_leftMotor_1, m_leftMotor_2);
  private final MotorControllerGroup m_rightMotor = new MotorControllerGroup(m_rightMotor_1, m_rightMotor_2);

  private final MotorControllerGroup e_motors = new MotorControllerGroup(e_motor_1, e_motor_2);
  private final DoubleSolenoid l_intake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.L_PNEWUMATIC_ID_1, RobotMap.L_PNEWUMATIC_ID_2);
    //r_intake = new DoubleSolenoid(null, RobotMap.R_PNEWUMATIC_ID_1, RobotMap.R_PNEWUMATIC_ID_2);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  private final Encoder elevator_encoder = new Encoder(0, 1);

  private final Encoder drive_encode_l = new Encoder(2, 3);
  private final Encoder drive_encode_r = new Encoder(8, 9);

  private final PigeonIMU _pigeon = new PigeonIMU(0);

  double[] ypr = new double[3];

  // private final RelativeEncoder shoulder_encoder = shoulder.getEncoder();
  private final RelativeEncoder elbow_encoder = elbow.getEncoder();


  public static final DriverController d_control = new DriverController(ControllerMap.DRIVER_PORT);
  public static final OperatorController o_control = new OperatorController(ControllerMap.OPERATOR_PORT);

  private static final String kDefaultAuto = "Default";
  private static final String kMiddleAuto = "Middle";

  private int current_timer = 0;

  private double global_current_distance;

  private final SendableChooser<String> chooser = new SendableChooser<>();

  private final AutoBalancer balancer = new AutoBalancer(m_robotDrive, ypr);

  private int seq = 0;

  @Override
  public void robotInit() {
    m_leftMotor.setInverted(true);
//
    m_leftMotor_1.setIdleMode(IdleMode.kBrake);
    m_leftMotor_2.setIdleMode(IdleMode.kBrake);
    m_rightMotor_1.setIdleMode(IdleMode.kBrake);
    m_rightMotor_2.setIdleMode(IdleMode.kBrake);

    drive_encode_r.setReverseDirection(true);

    chooser.setDefaultOption("Nothing", kDefaultAuto);
    chooser.addOption("Middle", kMiddleAuto);
    SmartDashboard.putData("Auto choices", chooser);


    drive_encode_l.setDistancePerPulse(RobotMap.CYCLES_PER_INCH);
    drive_encode_r.setDistancePerPulse(RobotMap.CYCLES_PER_INCH);

    

    elbow.setIdleMode(IdleMode.kBrake);


    SmartDashboard.updateValues();

  }


  @Override
  public void robotPeriodic() {



      super.robotPeriodic();

      //_pigeon.getYawPitchRoll(ypr);


      SmartDashboard.putNumber("drive_left_speed", m_leftMotor.get());
      SmartDashboard.putNumber("drive_right_speed", m_rightMotor.get());

      SmartDashboard.putNumber("shoulder_speed", shoulder.get());
      SmartDashboard.putNumber("elbow_speed", elbow.get());
      SmartDashboard.putNumber("elbow position", elbow_encoder.getPosition());

      SmartDashboard.putNumber("elevator_speed", elevator_encoder.getRaw());

      SmartDashboard.putNumber("drive_encode_ticks_l", drive_encode_l.getRaw());
      SmartDashboard.putNumber("drive_encode_ticks_distance_l", drive_encode_l.getDistance());
      
      SmartDashboard.putNumber("drive_encode_yaw", ypr[1]);

      SmartDashboard.putNumber("drive_encode_ticks_r", drive_encode_r.getRaw());
      SmartDashboard.putNumber("drive_encode_ticks_distance_r", drive_encode_r.getDistance());

      //SmartDashboard.putBoolean("elevator top limit", e_top_limit.get());      
      //SmartDashboard.putBoolean("elevator bottom limit", e_bottom_limit.get());

      SmartDashboard.putString("intake left", l_intake.get().toString());
      //SmartDashboard.putString("intake right", r_intake.get().toString());
    }

  @Override
  public void teleopInit() {
    elbow_encoder.setPosition(0);
    
    drive_encode_r.reset();
    drive_encode_l.reset();
  }

  @Override
  public void teleopPeriodic() {

  }


  public void autonomousInit(){
      current_timer = 0;
      elbow_encoder.setPosition(0);

      drive_encode_l.reset();
      drive_encode_r.reset();
      seq = 0;


    balancer.balancerInit();
  }

  public void autonomousPeriodic(){
//
  //  int target_milliseconds = 1000;
  //  
  //  if(current_timer < 500){
  //    m_robotDrive.arcadeDrive(1, 0);
  //  }
    if (current_timer < 1500){
      m_robotDrive.arcadeDrive(-0.7, 0);
    }
    else{
      m_robotDrive.arcadeDrive(0, 0);
    }
//
    current_timer += 20;
  }

  //public void drive(double distance, double speed){
  //  double current_distance = 0;
  //  
  //  while(distance > current_distance){
//
  //  }
  //}

  public boolean driveWithDistance(double current, double target, double speed){
    if(current < target){
      
      m_robotDrive.arcadeDrive(speed, 0);

      global_current_distance = drive_encode_r.getDistance();
      System.out.println("Driving forward! "+ global_current_distance);
      return false;
    }
    else{
      m_robotDrive.arcadeDrive(0, 0);
      return true;
    }
  }

  public boolean driveBackWithDistance(double current, double target, double speed){
    if(current > target){
      m_robotDrive.arcadeDrive(-speed, 0);

      global_current_distance = drive_encode_r.getDistance();

      System.out.println("Driving back! "+ global_current_distance);
      return false;
    }
    else{
      m_robotDrive.arcadeDrive(0, 0);
      return true;
    }
  }

  public void SetArmDown(){
    double target, current, error, kP;
    kP = -1/12;
    target = -12.78; //current 1000
    current = elbow_encoder.getPosition();

    error = current - target; //-800

    SmartDashboard.putNumber("Error", error);
    
    elbow.set(error * kP);

  }

  public void SetArmUP(){
    double target, current, error, kP;
    kP = -1/12;
    target = 0; //current 1000
    current = elbow_encoder.getPosition();

    error = current - target; //-1000

    elbow.set(error * kP);
  }
}