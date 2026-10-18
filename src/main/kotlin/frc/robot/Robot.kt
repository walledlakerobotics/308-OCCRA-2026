package frc.robot

import edu.wpi.first.epilogue.Epilogue
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands

@Logged
class Robot : TimedRobot() {
    private var autonomousCommand = Commands.none()
    private val robotContainer = RobotContainer()

    init {
        HAL.report(
            FRCNetComm.tResourceType.kResourceType_Language,
            FRCNetComm.tInstances.kLanguage_Kotlin,
            0,
            WPILibVersion.Version
        )

        DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog())

        Epilogue.bind(this)
        
        if (isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true)
        }
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun disabledExit() {}

    override fun autonomousInit() {
        autonomousCommand = robotContainer.autonomousCommand

        CommandScheduler.getInstance().schedule(autonomousCommand)
    }

    override fun autonomousPeriodic() {}

    override fun autonomousExit() {}

    override fun teleopInit() {
        CommandScheduler.getInstance().cancel(autonomousCommand)
    }

    override fun teleopPeriodic() {}

    override fun teleopExit() {}

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {}

    override fun testExit() {}
    
    override fun simulationInit() {}
    
    override fun simulationPeriodic() {}

    override fun endCompetition() {
        super.endCompetition()
        
        // Close all subsystems and resources
        robotContainer.close()
    }
}
