package org.firstinspires.ftc.teamcode.common.utils

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.util.ElapsedTime

class DeltaTime : SubsystemBase() {
    val timer: ElapsedTime = ElapsedTime()
    var hasStarted = false

    val deltaTime: Double
        get() = if (hasStarted) timer.seconds() else 0.0

    override fun periodic() {
        hasStarted = true
        timer.reset()
    }
}