package frc.robot.subsystems

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.kFeederCANID
import frc.robot.constants.kFeederMotorConfig

@Logged
class Feeder : SubsystemBase(), AutoCloseable {
    private val motor = SparkMax(kFeederCANID, SparkLowLevel.MotorType.kBrushed)

    init {
        motor.configure(kFeederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    val feed: Command
        get() = startEnd({
            motor.set(1.0)
        }, {
            motor.set(0.0)
        })

    val reverse: Command
        get() = startEnd({
            motor.set(-1.0)
        }, {
            motor.set(0.0)
        })

    override fun close() {
        motor.close()
    }
}