package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.constants.kTranslationFactor
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.subsystems.drivetrain.Vision

fun Drivetrain.alignToPose(
    pose: () -> Pose2d
): Command {
    val xController = PIDController(8.0, 0.0, 0.0)
    val yController = PIDController(8.0, 0.0, 0.0)
    val thetaController = ProfiledPIDController(
        5.0, 0.0, 0.0,
        TrapezoidProfile.Constraints(8.0, 4.0)
    )
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI)
    
    var targetPose: Pose2d

    SmartDashboard.putData("Alignment/X", xController)
    SmartDashboard.putData("Alignment/Y", yController)
    SmartDashboard.putData("Alignment/Theta", thetaController)
    
    return run {
        val currentPose = position
        targetPose = pose()
        
        val xSpeed = xController.calculate(currentPose.x, targetPose.x)
        val ySpeed = yController.calculate(currentPose.y, targetPose.y)
        val thetaSpeed = thetaController.calculate(currentPose.rotation.radians, targetPose.rotation.radians)
        
        drive(ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), true)
    }
}

fun Drivetrain.alignToHeading(
    x: () -> Double,
    y: () -> Double,
    targetHeading: () -> Rotation2d
): Command {
    val thetaController = ProfiledPIDController(
        5.0, 0.0, 0.0,
        TrapezoidProfile.Constraints(8.0, 4.0)
    )

    thetaController.enableContinuousInput(-Math.PI, Math.PI)

    var targetHeading: Rotation2d

    return run {
        targetHeading = targetHeading()

        val xSpeed = x() * kTranslationFactor
        val ySpeed = y() * kTranslationFactor
        val thetaSpeed = thetaController.calculate(heading.radians, targetHeading.radians)

        drive(
            ChassisSpeeds(
                xSpeed,
                ySpeed,
                thetaSpeed
            ),
            true
        )
    }
}

fun Drivetrain.alignToBall(
    vision: Vision,
    x: () -> Double,
    y: () -> Double
): Command {
    var currentTarget: Translation2d? = null

    return faceToward(x, y) {
        currentTarget = vision.getClosestBallTo(position, currentTarget)

        currentTarget
    }
}

fun Drivetrain.faceToward(
    x: () -> Double,
    y: () -> Double,
    target: () -> Translation2d?
) = alignToHeading(x, y) {
    val target = target() ?: return@alignToHeading heading
    val targetOffset = target - position.translation

    targetOffset.angle
}