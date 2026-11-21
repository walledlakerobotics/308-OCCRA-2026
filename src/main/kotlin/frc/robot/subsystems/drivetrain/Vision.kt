package frc.robot.subsystems.drivetrain

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import frc.robot.constants.robotToCamera
import org.photonvision.PhotonCamera
import org.photonvision.PhotonUtils
import org.photonvision.targeting.PhotonPipelineResult
import kotlin.jvm.optionals.getOrNull

@Logged
class Vision : AutoCloseable {
    private val camera = PhotonCamera("Intake")

    val cameraMatrix
        get() = camera.cameraMatrix.getOrNull() ?: Matrix(Nat.N3(), Nat.N3())

    val fx
        get() = cameraMatrix[0, 0]
    val fy
        get() = cameraMatrix[1, 1]
    val cx
        get() = cameraMatrix[0, 2]
    val cy
        get() = cameraMatrix[1, 2]

    var latestResult: PhotonPipelineResult? = null
        private set

    fun getBallPositions(robotPose: Pose2d) = buildList {
        for (target in latestResult?.targets ?: emptyList()) {
            val translation = PhotonUtils.estimateCameraToTargetTranslation(
                PhotonUtils.calculateDistanceToTargetMeters(
                    robotToCamera.z,
                    Units.inchesToMeters(9.5 / 2),
                    Units.degreesToRadians(-10.0),
                    Units.degreesToRadians(target.pitch)
                ),
                Rotation2d.fromDegrees(target.yaw)
            ) + robotToCamera.toTranslation2d()

            val transform = Transform2d(
                translation,
                Rotation2d.kZero
            )

            val position = robotPose + transform

            add(position.translation)
        }
    }

    fun getClosestBallTo(robotPose: Pose2d, targetPose: Translation2d? = null): Translation2d? {
        val targetPose = targetPose ?: robotPose.translation

        val ballPositions = getBallPositions(robotPose)
        return ballPositions.minByOrNull { it.getSquaredDistance(targetPose) }
    }

    fun update() {
        latestResult = camera.allUnreadResults.lastOrNull() ?:
            if (camera.isConnected) latestResult else null
    }

    override fun close() {
        camera.close()
    }
}