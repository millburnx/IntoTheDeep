package org.firstinspires.ftc.teamcode.common.utils

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.qualcomm.robotcore.util.ElapsedTime

@Config
object MotorTestPIDConfig {
    @JvmField
    var kP = 0.0

    @JvmField
    var kI = 0.0

    @JvmField
    var kD = 0.0

    @JvmField
    var kF = 0.0

    @JvmField
    var target = 0.0
}

class PID(
    val maxIntegralSum: Double = 50.0
) {
    private var target: Double = 0.0
    private var lastTarget: Double = 0.0

    private var lastError: Double = 0.0
    private var integralSum: Double = 0.0

    private val a: Double = 0.8
    private var previousFilterEstimate: Double = 0.0
    private var currentFilterEstimate: Double = 0.0

    private val deltaTimer: ElapsedTime = ElapsedTime()

    fun reset() {
        deltaTimer.reset()
    }

    fun calc(target: Double, current: Int): Double {
        val kp = MotorTestPIDConfig.kP
        val ki = MotorTestPIDConfig.kI
        val kd = MotorTestPIDConfig.kD

        val error = target - current
        val errorChange = error - lastError

        currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange
        previousFilterEstimate = currentFilterEstimate

        val derivative = currentFilterEstimate / deltaTimer.seconds()
        integralSum += (error * deltaTimer.seconds())

        integralSum = integralSum.clamp(-maxIntegralSum, maxIntegralSum)

        if (target != lastTarget) {
            integralSum = 0.0
        }

        val power = (kp * error) + (ki * integralSum) + (kd * derivative)
        lastError = error
        lastTarget = target

        deltaTimer.reset()
        return power
    }
}