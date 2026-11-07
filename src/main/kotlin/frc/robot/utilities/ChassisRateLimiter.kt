package frc.robot.utilities

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

class ChassisRateLimiter(val maxTranslationalAcceleration: Double, val maxAngularAcceleration: Double) {
    private var lastSpeeds = ChassisSpeeds()
    private val timer = Timer().apply { start() }
    private var lastTime = 0.0


    fun calculate(desiredSpeeds: ChassisSpeeds): ChassisSpeeds {
        val currentTime = timer.get()
        val dt = currentTime - lastTime

        val result: ChassisSpeeds

        if (dt <= 0.0) {
            result = desiredSpeeds
        } else {
            val desiredAcceleration = hypot(
                desiredSpeeds.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond,
                desiredSpeeds.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond
            ) / dt
            val obtainableAcceleration =
                MathUtil.clamp(desiredAcceleration, 0.0, maxTranslationalAcceleration)
            val theta = atan2(
                desiredSpeeds.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond,
                desiredSpeeds.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond
            )
            val desiredOmegaAcceleration = (desiredSpeeds.omegaRadiansPerSecond - lastSpeeds.omegaRadiansPerSecond) / dt
            val obtainableOmegaAcceleration = MathUtil.clamp(
                desiredOmegaAcceleration,
                -maxAngularAcceleration,
                maxAngularAcceleration
            )
            result = ChassisSpeeds(
                lastSpeeds.vxMetersPerSecond + cos(theta) * obtainableAcceleration * dt,
                lastSpeeds.vyMetersPerSecond + sin(theta) * obtainableAcceleration * dt,
                lastSpeeds.omegaRadiansPerSecond + obtainableOmegaAcceleration * dt
            )
        }

        lastTime = currentTime
        lastSpeeds = result

        return result
    }
}