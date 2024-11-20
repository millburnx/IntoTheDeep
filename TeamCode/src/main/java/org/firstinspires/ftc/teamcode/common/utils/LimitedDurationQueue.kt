package org.firstinspires.ftc.teamcode.common.utils

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.util.ElapsedTime

class LimitedDurationQueue<T>(var lifespan: Double, var threshold: Double = 60.0) : SubsystemBase() {
    private val internalValues: MutableList<Pair<Double, T>> = mutableListOf()
    private val timer = ElapsedTime()

    val values: List<T>
        get() = internalValues.map { it.second }

    fun add(element: T) {
        internalValues.add(timer.milliseconds() to element)
    }

    fun clear() {
        timer.reset()
        internalValues.clear()
    }

    fun isSaturated(): Boolean {
        if (internalValues.size < 2) return false
        return internalValues.first().first < timer.milliseconds() - lifespan + threshold
    }

    override fun periodic() {
        internalValues.removeIf {
            it.first < timer.milliseconds() - lifespan
        }
    }
}