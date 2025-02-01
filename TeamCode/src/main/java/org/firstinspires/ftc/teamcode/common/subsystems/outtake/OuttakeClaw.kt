package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init

@Config
class OuttakeClaw(
    val robot: Robot,
) : Subsystem() {
    inner class JSONSubsystem : com.millburnx.jsoncommands.Subsystem {
        override val type = "Subsystem/Outtake/Claw"

        override fun generate(): SubsystemBase = this@OuttakeClaw
    }

    val clawServo: ServoImplEx = (robot.hardware["outtakeClaw"] as ServoImplEx).apply { init() }

    var isOpen = true

    fun open() {
        isOpen = true
    }

    fun close() {
        isOpen = false
    }

    override fun init() {
        periodic()
    }

    override fun periodic() {
        clawServo.position = if (isOpen) open else closed
    }

    companion object {
        @JvmField
        var open: Double = 0.4

        @JvmField
        var closed: Double = 0.05
    }
}
