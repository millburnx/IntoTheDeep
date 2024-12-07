package org.firstinspires.ftc.teamcode.rr.util

import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

/**
 * Wraps a motor instance to provide corrected velocity counts and allow reversing independently of the corresponding
 * slot's motor direction
 */
class Encoder @JvmOverloads constructor(
    private val motor: DcMotorEx,
    private val clock: NanoClock = NanoClock.system()
) {
    enum class Direction(private val multiplier: Int) {
        FORWARD(1),
        REVERSE(-1);

        fun getMultiplier(): Int {
            return multiplier
        }
    }

    var direction: Direction = Direction.FORWARD

    private var lastPosition: Int = 0
    private var velocityEstimateIdx = 0
    private val velocityEstimates: DoubleArray = DoubleArray(3)
    private var lastUpdateTime: Double = clock.seconds()

    private fun getMultiplier(): Int {
        return direction.getMultiplier() * (if (motor.direction == DcMotorSimple.Direction.FORWARD) 1 else -1)
    }

    /**
     * Gets the position from the underlying motor and adjusts for the set direction.
     * Additionally, this method updates the velocity estimates used for compensated velocity
     *
     * @return encoder position
     */
    fun getCurrentPosition(): Int {
        val multiplier = getMultiplier()
        val currentPosition = motor.currentPosition * multiplier
        if (currentPosition != lastPosition) {
            val currentTime = clock.seconds()
            val dt = currentTime - lastUpdateTime
            velocityEstimates[velocityEstimateIdx] = (currentPosition - lastPosition) / dt
            velocityEstimateIdx = (velocityEstimateIdx + 1) % 3
            lastPosition = currentPosition
            lastUpdateTime = currentTime
        }
        return currentPosition
    }

    /**
     * Gets the velocity directly from the underlying motor and compensates for the direction
     * See [.getCorrectedVelocity] for high (>2^15) counts per second velocities (such as on REV Through Bore)
     *
     * @return raw velocity
     */
    fun getRawVelocity(): Double {
        val multiplier = getMultiplier()
        return motor.velocity * multiplier
    }

    /**
     * Uses velocity estimates gathered in [.getCurrentPosition] to estimate the upper bits of velocity
     * that are lost in overflow due to velocity being transmitted as 16 bits.
     * CAVEAT: must regularly call [.getCurrentPosition] for the compensation to work correctly.
     *
     * @return corrected velocity
     */
    fun getCorrectedVelocity(): Double {
        val median = if (velocityEstimates[0] > velocityEstimates[1]) max(
            velocityEstimates[1],
            min(velocityEstimates[0], velocityEstimates[2])
        ) else max(velocityEstimates[0], min(velocityEstimates[1], velocityEstimates[2]))
        return inverseOverflow(getRawVelocity(), median)
    }

    companion object {
        private const val CPS_STEP = 0x10000

        private fun inverseOverflow(input: Double, estimate: Double): Double {
            // convert to uint16
            var real = input.toInt() and 0xffff
            // initial, modulo-based correction: it can recover the remainder of 5 of the upper 16 bits
            // because the velocity is always a multiple of 20 cps due to Expansion Hub's 50ms measurement window
            real += ((real % 20) / 4) * CPS_STEP
            // estimate-based correction: it finds the nearest multiple of 5 to correct the upper bits by
            real += (((estimate - real) / (5 * CPS_STEP)).roundToInt() * 5 * CPS_STEP)
            return real.toDouble()
        }
    }
}
