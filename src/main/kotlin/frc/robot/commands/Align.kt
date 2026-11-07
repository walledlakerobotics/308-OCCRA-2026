package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.constants.kTranslationFactor
import frc.robot.subsystems.drivetrain.Drivetrain
import org.photonvision.targeting.PhotonTrackedTarget

fun Drivetrain.alignToPose(
    poseSupplier: () -> Pose2d
): Command {
    val xController = PIDController(8.0, 0.0, 0.0)
    val yController = PIDController(8.0, 0.0, 0.0)
    val thetaController = ProfiledPIDController(
        5.0, 0.0, 0.0,
        TrapezoidProfile.Constraints(8.0, 4.0)
    )
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI)
    
    var targetPose = poseSupplier()

    SmartDashboard.putData("Alignment/X", xController)
    SmartDashboard.putData("Alignment/Y", yController)
    SmartDashboard.putData("Alignment/Theta", thetaController)
    
    return run {
        val currentPose = position
        targetPose = poseSupplier()
        
        val xSpeed = xController.calculate(currentPose.x, targetPose.x)
        val ySpeed = yController.calculate(currentPose.y, targetPose.y)
        val thetaSpeed = thetaController.calculate(currentPose.rotation.radians, targetPose.rotation.radians)
        
        drive(ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), true)
    }
}

fun Drivetrain.alignToHeading(
    headingSupplier: () -> Rotation2d,
    x: () -> Double,
    y: () -> Double
): Command {
    val thetaController = ProfiledPIDController(
        5.0, 0.0, 0.0,
        TrapezoidProfile.Constraints(8.0, 4.0)
    )

    thetaController.enableContinuousInput(-Math.PI, Math.PI)

    var targetHeading = headingSupplier()

    SmartDashboard.putData("Alignment/Rot/Theta", thetaController)

    return run {
        val currentHeading = heading
        targetHeading = headingSupplier()

        val xSpeed = x() * kTranslationFactor
        val ySpeed = y() * kTranslationFactor
        val thetaSpeed = thetaController.calculate(currentHeading.radians, targetHeading.radians)

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

fun Drivetrain.alignToTarget(
    target: () -> PhotonTrackedTarget?,
    x: () -> Double,
    y: () -> Double,
    theta: () -> Double
): Command {
    var yaw = 0.0

    return run {
        val xSpeed = x() * kTranslationFactor
        val ySpeed = y() * kTranslationFactor

        val target = target()

        if (target != null) {
            yaw = Units.degreesToRadians(target.yaw)
        }

        val thetaSpeed = -yaw * 5.0

        drive(
            ChassisSpeeds(
                xSpeed,
                ySpeed,
                thetaSpeed
            ),
            false
        )
    }
}