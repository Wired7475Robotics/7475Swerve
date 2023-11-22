package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controll;
import frc.robot.Robot;

public class TeleopDrive extends CommandBase{
    double leftStickX;
    double leftStickY;
    double rightStickX;
    double rightStickY;
    double targetAngle;
    double currentAngle;
    double lastAngle = 0;

    final double FASTCOEF = 1.75;
    final double SLOWCOEF = 0.5;
    final double DEGREEINCREMEMNT = 1;
    

    public TeleopDrive(){
        addRequirements(Robot.drive);
    }
    
    public static double mod(double a, double b) { //booleg modulo function, works same as in python
		double c = a % b;
		c = c < 0? c + b : c;
		return c;
		}

    @Override
    public void initialize() {
        //Set PID values and enable PID
    }
    @Override
    public void execute() {
        //System.out.println("Exectuting drive command");
        //Get left stick values and calculate target angle
        leftStickY = -Robot.controlers.getDriveLeftStick(Controll.Y);
        leftStickX = -Robot.controlers.getDriveLeftStick(Controll.X);
        
        
        currentAngle = mod(Robot.navx.getAngle(), 360); //modulo starting angle
        targetAngle = Math.round(mod(Math.toDegrees(Math.atan2(leftStickY,leftStickX)) - 90, 360)/DEGREEINCREMEMNT)*DEGREEINCREMEMNT; //get target angle, between 0 and 360 deg
        
        double deadzone=0.9; //deadzone, set this fittingly for the flight sticks!
        if (Math.abs(leftStickY) < deadzone && Math.abs(leftStickX) < deadzone) { //prevent turning when in deadzone
			targetAngle = lastAngle;
		} else {
            lastAngle = targetAngle;
        }

        //Calculate shortest distance to target angle
		double diff = currentAngle - targetAngle;
		diff = mod( (diff + 180) , 360 ) - 180; //equals between -180 and 180
        
        //Send calculated rotation velocity to driveTrain
        Robot.drive.setRotationVelocity((diff/180)*16); //should equal between -4 and 4 (for now)
        
        //print info
        System.out.println("Target Angle: "+ targetAngle + " Current Angle: " + currentAngle + " Difference: " + diff + " Output: " + (diff/180));

        rightStickX = Robot.controlers.getOpLeftStick(Controll.X);
        rightStickY = Robot.controlers.getOpLeftStick(Controll.Y);
        
        //Add a fast/Slow mode that depends on button presses
        rightStickY = Controll.getOpA()? rightStickY * FASTCOEF : Controll.getOpB()? rightStickY * SLOWCOEF : rightStickY;
        rightStickX = Controll.getOpA()? rightStickX * FASTCOEF : Controll.getOpB()? rightStickX * SLOWCOEF : rightStickX;
        
        System.out.println("Right Stick: " + rightStickX + "," +  rightStickY);

        Robot.drive.setTranslationVelocity(rightStickX, rightStickY);
    }
}
