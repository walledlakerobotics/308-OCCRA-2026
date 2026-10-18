package frc.robot.subsystems

import com.ctre.phoenix6.controls.VelocityDutyCycle
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
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

        SmartDashboard.putNumber("Shooter/ShootingVelocity", 100.0)
    }

    fun shoot(velocity: Double) {
        shooterMotor.setControl(velocityRequest.withVelocity(velocity))
    }

    fun shoot(): Command {
        return startEnd({
            shoot(SmartDashboard.getNumber("Shooter/ShootingVelocity", 100.0))
        }, {
            stop()
        })
    }
    
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

    override fun close() {
        shooterMotor.close()
    }
}