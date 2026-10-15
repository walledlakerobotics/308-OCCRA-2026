package frc.robot.subsystems

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.kLeftIndexerCANID
import frc.robot.constants.kLeftIndexerMotorConfig
import frc.robot.constants.kRightIndexerCANID
import frc.robot.constants.kRightIndexerMotorConfig

@Logged
class Indexer : SubsystemBase() {
    val leftMotor = SparkMax(kLeftIndexerCANID, SparkLowLevel.MotorType.kBrushed)
    val rightMotor = SparkMax(kRightIndexerCANID, SparkLowLevel.MotorType.kBrushed)

    init {
        leftMotor.configure(kLeftIndexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        rightMotor.configure(kRightIndexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }
    
    val indexer: Command
        get() = startEnd({
                leftMotor.set(1.0)
                rightMotor.set(1.0)
            }, {
                leftMotor.set(0.0)
                rightMotor.set(0.0)
            })
}