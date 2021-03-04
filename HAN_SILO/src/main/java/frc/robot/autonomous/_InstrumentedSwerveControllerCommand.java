/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.autonomous;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.ReflectingCSVWriter;
import frc.robot.util.AutoSwerveDebug;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController ({@link ProfiledPIDController}) to follow a trajectory
 * {@link Trajectory} with a swerve drive.
 *
 * <p>This command outputs the raw desired Swerve Module States ({@link SwerveModuleState})
 * in an array. The desired wheel and module rotation velocities should be taken
 * from those and used in velocity PIDs.
 *
 * <p>The robot angle controller does not follow the angle given by
 * the trajectory but rather goes to the angle given in the final state of the trajectory.
 */

@SuppressWarnings("MemberName")
public class _InstrumentedSwerveControllerCommand extends CommandBase {
  private final Timer m_timer = new Timer();
  private Pose2d m_finalPose;

  private Trajectory m_trajectory;
  private Supplier<Pose2d> m_pose;
  private SwerveDriveKinematics m_kinematics;
  private PIDController m_xController;
  private PIDController m_yController;
  private ProfiledPIDController m_thetaController;
  private Consumer<SwerveModuleState[]> m_outputModuleStates;
  private ReflectingCSVWriter csvWriter;
  private Double endingHeading;
  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The rotation controller will calculate the rotation based on the final pose
   * in the trajectory, not the poses at each time step.
   *
   * @param trajectory         The trajectory to follow.
   * @param pose               A function that supplies the robot pose - use one of
   *                           the odometry classes to provide this.
   * @param kinematics         The kinematics for the robot drivetrain.
   * @param xController        The Trajectory Tracker PID controller
   *                           for the robot's x position.
   * @param yController        The Trajectory Tracker PID controller
   *                           for the robot's y position.
   * @param thetaController    The Trajectory Tracker PID controller
   *                           for angle for the robot.
   * @param outputModuleStates The raw output module states from the
   *                           position controllers.
   * @param requirements       The subsystems to require.
   */

  @SuppressWarnings("ParameterName")
  public _InstrumentedSwerveControllerCommand(
                               ReflectingCSVWriter<AutoSwerveDebug> csvWriter,
                               Trajectory trajectory,
                               Supplier<Pose2d> pose,
                               SwerveDriveKinematics kinematics,
                               PIDController xController,
                               PIDController yController,
                               ProfiledPIDController thetaController,

                               Consumer<SwerveModuleState[]> outputModuleStates,
                               Subsystem... requirements) {
    this.csvWriter = csvWriter;
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
    m_pose = requireNonNullParam(pose, "pose", "SwerveControllerCommand");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveControllerCommand");

    m_xController = requireNonNullParam(xController,
      "xController", "SwerveControllerCommand");
    m_yController = requireNonNullParam(yController,
      "xController", "SwerveControllerCommand");
    m_thetaController = requireNonNullParam(thetaController,
      "thetaController", "SwerveControllerCommand");

    m_outputModuleStates = requireNonNullParam(outputModuleStates,
      "leftFrontOutput", "SwerveControllerCommand");
    addRequirements(requirements);
  }

  @SuppressWarnings("ParameterName")
  public _InstrumentedSwerveControllerCommand(
                               ReflectingCSVWriter<AutoSwerveDebug> csvWriter,
                               Trajectory trajectory,
                               double endingHeading, //this is used to "fix up" a supplied trajectory
                               Supplier<Pose2d> pose,
                               SwerveDriveKinematics kinematics,
                               PIDController xController,
                               PIDController yController,
                               ProfiledPIDController thetaController,

                               Consumer<SwerveModuleState[]> outputModuleStates,
                               Subsystem... requirements) {
    this.csvWriter = csvWriter;
    this.endingHeading = endingHeading;
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
    m_pose = requireNonNullParam(pose, "pose", "SwerveControllerCommand");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveControllerCommand");

    m_xController = requireNonNullParam(xController,
      "xController", "SwerveControllerCommand");
    m_yController = requireNonNullParam(yController,
      "xController", "SwerveControllerCommand");
    m_thetaController = requireNonNullParam(thetaController,
      "thetaController", "SwerveControllerCommand");

    m_outputModuleStates = requireNonNullParam(outputModuleStates,
      "leftFrontOutput", "SwerveControllerCommand");
    addRequirements(requirements);
  }


  @Override
  public void initialize() {
    // Sample final pose to get robot rotation
    m_finalPose = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters;

    m_timer.reset();
    m_timer.start();
  }


  @Override
  @SuppressWarnings("LocalVariableName")
  public void execute() {
    double curTime = m_timer.get();

    var desiredState = m_trajectory.sample(curTime);
    var desiredPose = desiredState.poseMeters;

  
    
    

    Pose2d poseError = desiredPose.relativeTo(m_pose.get());

    csvWriter.add(new AutoSwerveDebug(curTime, 
                                   desiredPose.getTranslation().getX(),
                                   desiredPose.getTranslation().getY(),
                                   desiredPose.getRotation().getDegrees(),
                                   m_pose.get().getTranslation().getX(),
                                   m_pose.get().getTranslation().getY(),
                                   m_pose.get().getRotation().getDegrees()));

 //  var feedForwardX = desiredState.poseMeters.getRotation().getSin()*desiredState.velocityMetersPerSecond;                              

 //  var feedForwardY = desiredState.poseMeters.getRotation().getCos()*desiredState.velocityMetersPerSecond;

    double targetXVel = m_xController.calculate(
        m_pose.get().getTranslation().getX(),
        desiredPose.getTranslation().getX());

    double targetYVel = m_yController.calculate(
        m_pose.get().getTranslation().getY(),
        desiredPose.getTranslation().getY());

    // The robot will go to the desired rotation of the final pose in the trajectory,
    // not following the poses at individual states.
    double targetAngularVel = m_thetaController.calculate(
        m_pose.get().getRotation().getRadians(),
        endingHeading == null ? m_finalPose.getRotation().getRadians() : Rotation2d.fromDegrees(endingHeading).getRadians());

    double vRef = desiredState.velocityMetersPerSecond;

    //targetXVel += vRef * poseError.getRotation().getCos();
   // targetYVel += vRef * poseError.getRotation().getSin();

    targetXVel += vRef * desiredState.poseMeters.getRotation().getCos();
    targetYVel += vRef * desiredState.poseMeters.getRotation().getSin();

    

    var targetChassisSpeeds = new ChassisSpeeds(targetXVel, targetYVel, targetAngularVel);

    var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

    m_outputModuleStates.accept(targetModuleStates);

  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
