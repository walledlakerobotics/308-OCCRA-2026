package frc.robot.subsystems.drivetrain

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.MecanumDriveKinematics
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.*

@Logged
class MecanumDrive : SubsystemBase(), AutoCloseable {
    private val frontLeftNeo = SparkMax(kFrontLeftMotorCANID, SparkLowLevel.MotorType.kBrushless)
    private val frontRightNeo = SparkMax(kFrontRightMotorCANID, SparkLowLevel.MotorType.kBrushless)
    private val backLeftNeo = SparkMax(kBackLeftMotorCANID, SparkLowLevel.MotorType.kBrushless)
    private val backRightNeo = SparkMax(kBackRightMotorCANID, SparkLowLevel.MotorType.kBrushless)

    private val kinematics = MecanumDriveKinematics(
        kFrontLeftNeoTranslation,
        kFrontRightNeoTranslation,
        kBackLeftNeoTranslation,
        kBackRightNeoTranslation
    )

    init {
        frontLeftNeo.configure(kLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        frontRightNeo.configure(kRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        backLeftNeo.configure(kLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        backRightNeo.configure(kRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }
    
    fun drive(speeds: ChassisSpeeds) {
        // gets kinematic velocity's
        val v = kinematics.toWheelSpeeds(speeds)

        // sets the set point.
        frontLeftNeo.closedLoopController.setSetpoint(v.frontLeftMetersPerSecond, SparkBase.ControlType.kVelocity)
        frontRightNeo.closedLoopController.setSetpoint(v.frontRightMetersPerSecond, SparkBase.ControlType.kVelocity)
        backRightNeo.closedLoopController.setSetpoint(v.rearRightMetersPerSecond, SparkBase.ControlType.kVelocity)
        backLeftNeo.closedLoopController.setSetpoint(v.rearLeftMetersPerSecond, SparkBase.ControlType.kVelocity)
    }

    fun drive(x: () -> Double, y: () -> Double, theta: () -> Double): Command {
        return run {
            drive(
                ChassisSpeeds(
                    x() * kSpeedFactor,
                    y() * kSpeedFactor,
                    theta() * kSpeedFactor
                )
            )
        }
    }

    fun stop() {
        drive(ChassisSpeeds())
    }
    
    override fun close() {
        frontLeftNeo.close()
        frontRightNeo.close()
        backLeftNeo.close()
        backRightNeo.close()

        driveOdometry.close()
    }
}