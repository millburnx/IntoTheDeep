package org.firstinspires.ftc.teamcode.common.utils

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.teamcode.common.subsystems.misc.DeltaTime

class SlewRateLimiter {
    companion object {
        fun limit(current: Double, target: Double, maxRate: Double): Double {
            val rate = target - current
            return current + rate.coerceIn(-maxRate, maxRate)
        }

        fun limit(current: Vec2d, target: Vec2d, maxRate: Double): Vec2d {
            return Vec2d(limit(current.x, target.x, maxRate), limit(current.y, target.y, maxRate))
        }
    }
}

// rate per second
class GamepadSRL(val gamepad: GamepadEx, val maxLeftRate: Double, val maxRightRate: Double, val deltaTime: DeltaTime) :
    SubsystemBase() {
    var leftStick: Vec2d = Vec2d(0.0, 0.0)
    var rightStick: Vec2d = Vec2d(0.0, 0.0)

    override fun periodic() {
        val currentLeft = Vec2d(gamepad.leftX, gamepad.leftY)
        val currentRight = Vec2d(gamepad.rightX, gamepad.rightY)
        leftStick = SlewRateLimiter.limit(leftStick, currentLeft, maxLeftRate * deltaTime.deltaTime)
        rightStick = SlewRateLimiter.limit(rightStick, currentRight, maxRightRate * deltaTime.deltaTime)
    }
}