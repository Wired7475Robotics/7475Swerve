package frc.robot.subsystems;
import java.io.File;
import java.io.IOException;

import edu.wpi.first.cscore.VideoListener;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controll;
import frc.robot.Robot;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

public class drivetrain extends SubsystemBase{
    private File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");

    public SwerveDrive swerveDrive;

    private Translation2d driveTranslation;
    private double speedCoef = 2.5;

    private double[] velocities = new double[3];

    public drivetrain(){
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive();
        } catch (Exception e) {
            System.err.println(e + "Failed to Init Swerve Drive");
            e.printStackTrace();
        }
    }

    public void setRotationVelocity(double value){
        velocities[2] = value;
    }

    public void setTranslationVelocity(double X, double Y){
        velocities[0] = X;
        velocities[1] = Y;
    }

    @Override
    public void periodic() {
        driveTranslation = new Translation2d(velocities[1]*speedCoef, velocities[0]*speedCoef);
        swerveDrive.drive(driveTranslation, velocities[2], true, false);
    }
}
