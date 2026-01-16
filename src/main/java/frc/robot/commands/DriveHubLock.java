// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class DriveHubLock extends Command {

  private final Drivebase drivebase;
  private final Supplier<double[]> speedXY;
  private final DriverStation.Alliance alliance = DriverStation.getAlliance().orElseThrow();

  private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(18, 18);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0, THETA_CONSTRAINTS);

  /** Creates a new Drive. */
  public DriveHubLock(Drivebase drivebase, Supplier<double[]> speedXY) {
    this.drivebase = drivebase;
    this.speedXY = speedXY;

    thetaController.setTolerance(Units.degreesToRadians(2));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset(drivebase.getPose().getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  private double thetaSpeed;
  private Pose2d goalPose;
  //TODO: SET FIELD TO 2026 WHEN UPDATED
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  @Override
  public void execute() {
    var xy = speedXY.get();

    if (alliance.equals(DriverStation.Alliance.Red))
    {
      //10
      Pose2d tag = aprilTagFieldLayout.getTagPose(10).orElseThrow().toPose2d();
      goalPose = new Pose2d(tag.getX() + Units.inchesToMeters(47.0/2), tag.getY(), tag.getRotation());
    }
    else if (alliance.equals(DriverStation.Alliance.Blue))
    {
      //26
      Pose2d tag = aprilTagFieldLayout.getTagPose(26).orElseThrow().toPose2d();
      goalPose = new Pose2d(tag.getX() - Units.inchesToMeters(47.0/2), tag.getY(), tag.getRotation());
    }
    else 
    {
      if (drivebase.getPose().getX() > 16.53)
      {
        Pose2d tag = aprilTagFieldLayout.getTagPose(26).orElseThrow().toPose2d();
        goalPose = new Pose2d(tag.getX() - Units.inchesToMeters(47.0/2), tag.getY(), tag.getRotation());
      }
      else 
      {
        Pose2d tag = aprilTagFieldLayout.getTagPose(10).orElseThrow().toPose2d();
        goalPose = new Pose2d(tag.getX() + Units.inchesToMeters(47.0/2), tag.getY(), tag.getRotation());
      }
    }

    Pose2d robotPose = drivebase.getPose();
    thetaController.setGoal(Math.atan((goalPose.getY() - robotPose.getY())/(goalPose.getX() - robotPose.getX())));
    thetaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());

    drivebase.defaultDrive(-xy[1], -xy[0], thetaSpeed);
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
