package org.firstinspires.ftc.teamcode.common.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Limelight(hardwareMap: HardwareMap, telemetry: Telemetry) : SubsystemBase() {
    val limelight = hardwareMap.get(Limelight3A::class.java, "limelight").apply {
        pipelineSwitch(0)
        start()
    }

    init {
        telemetry.msTransmissionInterval = 11
    }

    fun getResults(): LLResult? {
        return limelight.latestResult
    }

    fun updateRobotOrientation(yaw: Double) {
        limelight.updateRobotOrientation(yaw)
    }
}