package frc.robot.Controllers;

import edu.wpi.first.wpilibj.XboxController;

public class OperatorController extends XboxController {
/**
 * Controller Map:
 * 
 * Left Stick X:
 * Left Stick Y:
 * Left Stick Button:
 * 
 * Right Stick X:
 * Right Stick Y:
 * Right Stick Button:
 * 
 * Left Bumper: Toggle Left Intake
 * Right Bumper: Toggle Right Intake Piston
 * 
 * Left Trigger: 
 * Right Trigger: 
 * 
 * Button A:
 * Button B: 
 * Button X: Auto Down Elevator
 * Button Y: Auto Up Elevator
 * 
 * Button Select:
 * Button Start:
 * 
 * POV 0: Manual UP Elevator
 * POV 45:
 * POV 90: 
 * POV 135:
 * POV 180: Manual Down Elevator
 * POV 225:
 * POV 270:
 */

   public OperatorController(int port){
      super(port);
   }

   public boolean RetrieveIntake(){
      return getLeftBumper();
   }
   public boolean ReleaseIntake(){
     return getRightBumper();
   }  

   public double RotateIntake(){
      return getRightY();
   }


   public boolean setArmUpBind(){
      return getYButton();
   }
   public boolean setArmDownBind(){
      return getAButton();
   }
}
