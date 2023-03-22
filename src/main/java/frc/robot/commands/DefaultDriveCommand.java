package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

    public DriveSubsystem drive;
    
    public DefaultDriveCommand(DriveSubsystem _drive){
        super();
        drive = _drive;
        addRequirements(_drive);
    }

    @Override
    public void execute(){
        //if(Robot.d_control
        drive.standardDrive(Robot.d_control.getSpeed(), Robot.d_control.getRotation());
    }
}
