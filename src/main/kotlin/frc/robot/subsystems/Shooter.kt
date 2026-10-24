package frc.robot.subsystems

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.constants.kShootCANID
import frc.robot.constants.kShooterConfig
import frc.robot.constants.kShootingVelocity

@Logged
class Shooter : SubsystemBase(), AutoCloseable {
    private val velocityRequest = VelocityVoltage(0.0)
    private val voltageRequest = VoltageOut(0.0)

    private val shooterMotor = TalonFX(kShootCANID)

    init {
        shooterMotor.configurator.apply(kShooterConfig)

        SmartDashboard.putNumber("Shooter/ShootingVelocity", 62.0)
    }

    fun shoot(velocity: Double) {
        shooterMotor.setControl(velocityRequest.withVelocity(velocity))
    }

    val shoot: Command
        get() = startEnd({
            shoot(SmartDashboard.getNumber("Shooter/ShootingVelocity", 100.0))
        }, {
            stop()
        })

    val shootReverse: Command
        get() = startEnd({
            shoot(-5.0)
        }, {
            stop()
        })
    

    fun stop() {
        shoot(0.0)
    }

    override fun periodic() {
        SmartDashboard.putNumber("Shooter/Velocity", shooterMotor.velocity.valueAsDouble)
    }

    private val sysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(null, null, null) { state ->
            SignalLogger.writeString("sysid-test-state-shooter", state.toString())
        },
        SysIdRoutine.Mechanism({ volts ->
            shooterMotor.setControl(voltageRequest.withOutput(volts))
        }, null, this, "shooter")
    )

    val sysIdForwardQuasistatic: Command
        get() = sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)

    val sysIdReverseQuasistatic: Command
        get() = sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)

    val sysIdForwardDynamic: Command
        get() = sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)

    val sysIdReverseDynamic: Command
        get() = sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
        
    val atGoal: Boolean
        get() = shooterMotor.velocity.valueAsDouble >= SmartDashboard.getNumber("Shooter/ShootingVelocity", 62.0) * 0.95

    override fun close() {
        shooterMotor.close()
    }
}