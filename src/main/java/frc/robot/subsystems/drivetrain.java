package frc.robot.subsystems;
import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Controll;
import frc.robot.Robot;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

public class drivetrain {
    private File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");

    public SwerveDrive swerveDrive;

    private Translation2d driveTranslation;

    public drivetrain(){
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive();
        } catch (Exception e) {
            System.err.println(e + "Failed to Init Swerve Drive");
            e.printStackTrace();
        }
    }

    public void swervedrive(){
<<<<<<< Updated upstream
        driveTranslation = new Translation2d(Robot.controlers.getDriveRightStick(Controll.Y)*4, Robot.controlers.getDriveRightStick(Controll.X)*4);
        swerveDrive.drive(driveTranslation, Robot.controlers.getDriveLeftStick(Controll.X)*7, true, false);
=======
        driveTranslation = new Translation2d(Controll.getOpLeftStick(Controll.Y)*1, Controll.getOpLeftStick(Controll.X)*1);
        swerveDrive.drive(driveTranslation, Robot.controlers.getDriveLeftStick(Controll.X)*-2, true, false);
>>>>>>> Stashed changes
    }
}
