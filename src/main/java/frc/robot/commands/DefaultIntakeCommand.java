package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase {
    private IntakeSubsystem intake;
    private double DEADZONE = 0.1; 
    
    public DefaultIntakeCommand(IntakeSubsystem _intake){
        super();
        intake = _intake;
        addRequirements(_intake);

        
    }

    @Override
    public void execute(){
        if(Robot.o_control.RetrieveIntake()){
            intake.retrieveGamepiece();
        }
        else if(Robot.o_control.ReleaseIntake()){
            intake.releaseGamepiece();
        }


        if(Robot.o_control.RotateIntake() > DEADZONE && Robot.o_control.RotateIntake() < -DEADZONE){
            intake.retrieveGamepiece();
        }
    }
}
