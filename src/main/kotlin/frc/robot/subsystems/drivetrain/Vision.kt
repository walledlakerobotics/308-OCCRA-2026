package frc.robot.subsystems.drivetrain

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
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

    var results = emptyList<PhotonPipelineResult>()
        private set

    val latestResult
        get() = results.lastOrNull()

    val bestTarget
        get() = if (latestResult?.hasTargets() ?: false) latestResult?.bestTarget else null

    val bestTargetPose: Translation2d?
        get() {
            val target = bestTarget ?: return null

            return PhotonUtils.estimateCameraToTargetTranslation(
                PhotonUtils.calculateDistanceToTargetMeters(
                    0.0,
                    9.5/2,
                    0.0,
                    Units.degreesToRadians(target.pitch)
                ),
                Rotation2d.fromDegrees(target.yaw)
            )
        }

    fun update() {
        results = camera.allUnreadResults
    }

    override fun close() {
        camera.close()
    }
}