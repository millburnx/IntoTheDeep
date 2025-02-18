package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init

@Config
class Diffy(
    val robot: Robot,
) : Subsystem() {
    val leftServo: ServoImplEx = (robot.hardware["diffyLeft"] as ServoImplEx).apply { init() }
    val rightServo: ServoImplEx = (robot.hardware["diffyRight"] as ServoImplEx).apply { init(false) }

    // each only has an effective range of +/- 0.25 aka 0-0.5 or half
    // as you need to ensure that pitch + roll never exceeds the bounds of 0 to 1
    var pitch: Double = transferPitch // up and down, -1 to 1
    var roll: Double = transferRoll // rotate, -1 to 1

    override fun init() {
        periodic()
    }

    override fun periodic() {
        val pitchPosition = 0.5 + pitch / 4 // convert to 0.25 to 0.75
        val leftPosition = pitchPosition - roll / 4 // 0 to 1
        val rightPosition = pitchPosition + roll / 4 // 0 to 1
        leftServo.position = leftPosition
        rightServo.position = rightPosition
    }

    fun transfer() =
        InstantCommand({
            pitch = transferPitch
            roll = transferRoll
        })

    fun specimen() =
        InstantCommand({
            pitch = specimenPitch
            roll = specimenRoll
        })

    fun hover() =
        InstantCommand({
            pitch = hoverPitch
            roll = hoverRoll
        })

    fun pickup() =
        InstantCommand({
            pitch = pickupPitch
        })

    companion object {
        @JvmField
        var transferPitch = 0.8

        @JvmField
        var transferRoll = 1.0

        @JvmField
        var specimenPitch = 0.25

        @JvmField
        var specimenRoll = 1.0

        @JvmField
        var hoverPitch = -0.8

        @JvmField
        var hoverRoll = -1.0

        @JvmField
        var pickupPitch = -0.7

//        @JvmField
//        var pickupRoll = -0.025

        @JvmField
        var roll60: Double = -1 / 3.0

        @JvmField
        var roll120: Double = 1 / 3.0
    }
}
