package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.controller.PIDController
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.AxonCR
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

@Config
class Diffy(
    val robot: Robot,
) : Subsystem() {
    val leftServo = AxonCR(robot.hardware, "diffyLeft", "analog2")
    val rightServo = AxonCR(robot.hardware, "diffyRight", "analog1", false)

    // each only has an effective range of +/- 0.25 aka 0-0.5 or half
    // as you need to ensure that pitch + roll never exceeds the bounds of 0 to 1
    var pitch: Double = transferPitch // up and down, -1 to 1
    var roll: Double = transferRoll // rotate, -1 to 1

    override fun init() {
        periodic()
    }

    val pidLeft = PIDController(kP, kI, kD)
    val pidRight = PIDController(kP, kI, kD)

    override fun periodic() {
        pidLeft.setPID(kP, kI, kD)
        pidRight.setPID(kP, kI, kD)

        val pitchPosition = 0.5 + pitch / 4 // convert to 0.25 to 0.75
        val leftPosition = pitchPosition - roll / 4 // 0 to 1
        val rightPosition = pitchPosition + roll / 4 // 0 to 1

        leftServo.power = pidLeft.calculate(leftServo.position, leftPosition)
        rightServo.power = pidLeft.calculate(rightServo.position, rightPosition)
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
        var kP = 0.1

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0

        @JvmField
        var transferPitch = 0.95

        @JvmField
        var transferRoll = 0.95

        @JvmField
        var specimenPitch = 0.6

        @JvmField
        var specimenRoll = 1.0

        @JvmField
        var hoverPitch = -0.9

        @JvmField
        var hoverRoll = -1.0

        @JvmField
        var pickupPitch = -0.6
    }
}
