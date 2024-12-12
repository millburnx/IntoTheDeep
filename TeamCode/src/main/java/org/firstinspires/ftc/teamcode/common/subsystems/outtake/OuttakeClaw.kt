package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init

@Config
class OuttakeClaw(val robot: Robot) : Subsystem() {
    val clawServo: Servo = (robot.hardware["outtakeClaw"] as Servo).apply { init() }

    val isOpen = true

    override fun periodic() {
        clawServo.position = if (isOpen) open else closed
    }

    companion object {
        @JvmField
        var open: Double = 0.25

        @JvmField
        var closed: Double = 0.75
    }
}