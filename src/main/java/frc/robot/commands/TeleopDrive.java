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
    

    public TeleopDrive(){
        addRequirements(Robot.drive);
    }
    
    

    @Override
    public void initialize() {
        //Set PID values and enable PID
    }
    @Override
    public void execute() {
        //System.out.println("Exectuting drive command");
        //Get left stick values and calculate target angle
        leftStickX = Robot.controlers.getDriveLeftStick(Controll.X);
        leftStickY = Robot.controlers.getDriveLeftStick(Controll.Y);
        targetAngle = Math.toDegrees(Math.atan2(leftStickY,leftStickX));
        System.out.println("Target Angle: "+ targetAngle + " leftStickX: " + leftStickX + " leftStickY: " + leftStickY);
        currentAngle = Robot.navx.getAngle() % 360;
        currentAngle = currentAngle < 0? currentAngle + 360 : currentAngle;
        targetAngle = targetAngle < 0? targetAngle +360 : currentAngle;
        
        //Calculate shortest distance to target angle
        double diff = currentAngle - targetAngle;
        double absdiff = Math.abs(currentAngle - targetAngle);
        double absaltDiff = Math.abs(diff - 360);
        


        double output = absdiff < absaltDiff? diff : -1 * absaltDiff;
        output = diff < 0? output * -1 : output;  
        
        //Output calculated rotation velocity to driveTrain
        //System.out.println("Target Angle: "+ targetAngle + " Current Angle: " + currentAngle + " Difference: " + diff + " Output: " + output);
        Robot.drive.setRotationVelocity(-output/180);

        rightStickX = Robot.controlers.getOpLeftStick(Controll.X);
        rightStickY = Robot.controlers.getOpLeftStick(Controll.Y);
        Robot.drive.setTranslationVelocity(rightStickX, rightStickY);
    }
}
