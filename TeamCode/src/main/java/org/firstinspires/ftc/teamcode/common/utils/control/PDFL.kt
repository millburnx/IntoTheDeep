package org.firstinspires.ftc.teamcode.common.utils.control

import org.firstinspires.ftc.teamcode.common.utils.normalizeDegrees
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

class PDFL(
    var kP: Double,
    var kD: Double,
    var kF: Double,
    var kL: Double,
    var tolerance: Double,
    var iSquid: Boolean = false,
) {
    var lastError: Double? = 0.0
    var lastTarget: Double = 0.0

    fun reset() {
        lastError = 0.0
    }

    fun calculate(
        current: Double,
        target: Double,
        deltaTime: Double,
    ): Double {
        if (target != lastTarget) {
            reset()
            lastTarget = target
        }
        val error = target - current
        return calculate(error, deltaTime)
    }

    fun calculateHeading(
        current: Double,
        target: Double,
        deltaTime: Double,
    ): Double {
        if (target != lastTarget) {
            reset()
            lastTarget = target
        }
        val error = normalizeDegrees(target - current)
        return calculate(error, deltaTime)
    }

    fun calculate(
        error: Double,
        deltaTime: Double,
    ): Double {
        if (lastError == null) {
            lastError = error
        }

        val p = if (iSquid) sqrt(abs(p(error))) * sign(error) else p(error)
        val d = d(error, deltaTime)
        val f = f()
        val l = l(error)

        lastError = error

        if (abs(error) < tolerance) {
            return p + d + f
        }

        return p + d + f + l
    }

    fun p(error: Double) = kP * error

    fun d(
        error: Double,
        deltaTime: Double,
    ) = kD * (error - (lastError ?: error)) / deltaTime

    fun f() = kF

    fun l(error: Double) = kL * sign(error)
}
