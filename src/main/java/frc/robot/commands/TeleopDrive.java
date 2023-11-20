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
    PIDController pid = new PIDController(0.05, 0, 0);

    public TeleopDrive(){
        addRequirements(Robot.drive);
    }
    
    

    @Override
    public void initialize() {
        //Set PID values and enable PID
        pid.setSetpoint(0);
        pid.setTolerance(0.1);
    }
    @Override
    public void execute() {
        System.out.println("Exectuting drive command");
        //Get left stick values and calculate target angle
        leftStickX = Robot.controlers.getDriveLeftStick(Controll.X);
        leftStickY = Robot.controlers.getDriveLeftStick(Controll.Y);
        targetAngle = Math.toDegrees(Math.atan2(leftStickY,leftStickX));
        currentAngle = Robot.navx.getAngle() % 360;
        currentAngle = currentAngle < 0? currentAngle + 360 : currentAngle;
        targetAngle = targetAngle < 0? targetAngle +360 : currentAngle;
        
        //Calculate shortest distance to target angle
        double absdiff = Math.abs(currentAngle - targetAngle);
        double absaltDiff = Math.abs(absdiff - 360);
        double diff = currentAngle - targetAngle;
        
        /* Evan pseudo code
        if absdiff < absaltDiff:
            output = diff
        if absaltDiff < absdiff:
            output = -1 * absaltDiff
            if diff < 0:
                output = output * -1
        */

        
        
        //Output calculated rotation velocity to driveTrain
        pid.setSetpoint(targetAngle);
        //double output = pid.calculate(currentAngle);
        double output = 0; //JB pseudo-code into java; sorry if its busted
        if (absdiff < absaltDiff) {
            output = diff;
        }
        if (absaltDiff < absdiff) {
            output = -1 * absaltDiff;
            if (diff < 0) {
            output = output * -1;
            }
        }
        System.out.println("Target Angle: "+ targetAngle + " Current Angle: " + currentAngle + " Difference: " + diff + " Output: " + output);
        Robot.drive.setRotationVelocity(-output/360);

        rightStickX = Robot.controlers.getOpLeftStick(Controll.X);
        rightStickY = Robot.controlers.getOpLeftStick(Controll.Y);
        Robot.drive.setTranslationVelocity(rightStickX, rightStickY);
    }
}
