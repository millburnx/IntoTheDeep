package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.controller.PIDController
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.hardware.AxonCR

@Config
class Diffy(
    val robot: Robot,
) : Subsystem() {
    val leftServo = AxonCR(robot.hardware, "diffyLeft", "analog2", false)
    val rightServo = AxonCR(robot.hardware, "diffyRight", "analog1")

    var manualOverride: Boolean = false

    // each only has an effective range of +/- 0.25 aka 0-0.5 or half
    // as you need to ensure that pitch + roll never exceeds the bounds of 0 to 1
    var pitch: Double = transferPitch // up and down, -1 to 1
    var roll: Double = transferRoll // rotate, -1 to 1

    override fun init() {
//        periodic()
    }

    val pidLeft = PIDController(kP, kI, kD)
    val pidRight = PIDController(kP, kI, kD)

    override fun periodic() {
        if (manualOverride) {
            return
        }

        pidLeft.setPID(kP, kI, kD)
        pidRight.setPID(kP, kI, kD)

        val pitchPosition = pitch
        val leftPosition = pitchPosition - roll
        val rightPosition = pitchPosition + roll

        leftServo.power = pidLeft.calculate(leftServo.position, leftPosition)
        rightServo.power = pidLeft.calculate(rightServo.position, rightPosition)

//        leftServo.power = 0.0
//        rightServo.power = 0.0
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
        var kP = -1.0

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0

        @JvmField
        var transferPitch = 0.88

        @JvmField
        var transferRoll = 0.0

        @JvmField
        var specimenPitch = 0.64

        @JvmField
        var specimenRoll = 0.0

        @JvmField
        var hoverPitch = 0.4

        @JvmField
        var hoverRoll = 0.0

        @JvmField
        var pickupPitch = 0.4
    }
}
