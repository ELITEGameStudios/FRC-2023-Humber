package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class AutoBalancer {
    private DifferentialDrive drive;
    private double[] ypr;
    private int state;

    private double onChargeDegree;
    // private double levelDegree;

    public AutoBalancer(DifferentialDrive drive, double[] ypr){
        this.drive = drive;
        this.ypr = ypr;

        state = 0;
        onChargeDegree = 2;
        // levelDegree = 7;
    }

    //states
    /* 0 init
     * 1 too close to drive station
     * 2 too far from drive station
     * 3 level
     */
    public void balancerInit(){

    }

    public void balancerPeriodic(){
        switch(state){
            case 0: 
                if(ypr[1] < -onChargeDegree) {state = 1;} //too close
                else if(ypr[1] > onChargeDegree) {state = 2;} //too far
                else {state = 3;}
            break;
            case 1:
                drive.arcadeDrive(0.3, 0);
                if(ypr[1] > onChargeDegree) {state = 2;}
            break;
            case 2:
                drive.arcadeDrive(-0.3, 0);
                if(ypr[1] < -onChargeDegree) {state = 1;} //too close
            break;
        }
    }
}