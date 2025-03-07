package org.firstinspires.ftc.teamcode.common.utils.control

import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sqrt

@Suppress("ktlint:standard:no-consecutive-comments", "ktlint:standard:property-naming")
open class SQUIDFController(
    var p: Double,
    var i: Double,
    var d: Double,
    var f: Double,
    private var setPoint: Double = 0.0,
    private var measuredValue: Double = 0.0,
) {
    private var minIntegral: Double
    private var maxIntegral = 1.0

    /**
     * @return the positional error e(t)
     */
    var positionError: Double
        private set

    /**
     * @return the velocity error e'(t)
     */
    var velocityError: Double = 0.0
        private set

    private var totalError = 0.0
    private var prevErrorVal = 0.0

    private var errorTolerance_p = 0.05
    private var errorTolerance_v = Double.POSITIVE_INFINITY

    private var lastTimeStamp = 0.0
    var period: Double = 0.0
        private set

    /**
     * This is the full constructor for the PIDF controller. Our PIDF controller
     * includes a feed-forward value which is useful for fighting friction and gravity.
     * Our errorVal represents the return of e(t) and prevErrorVal is the previous error.
     *
     * @param setPoint The setpoint of the pid control loop.
     * @param measuredValue The measured value of he pid control loop. We want sp = pv, or to the degree
     * such that sp - pv, or e(t) < tolerance.
     */

    /**
     * The base constructor for the PIDF controller
     */
    init {
        minIntegral = -1.0

        positionError = setPoint - measuredValue
        reset()
    }

    fun reset() {
        totalError = 0.0
        prevErrorVal = 0.0
        lastTimeStamp = 0.0
    }

    /**
     * Sets the error which is considered tolerable for use with [.atSetPoint].
     *
     * @param positionTolerance Position error which is tolerable.
     */
    fun setTolerance(positionTolerance: Double) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY)
    }

    /**
     * Sets the error which is considered tolerable for use with [.atSetPoint].
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    fun setTolerance(
        positionTolerance: Double,
        velocityTolerance: Double,
    ) {
        errorTolerance_p = positionTolerance
        errorTolerance_v = velocityTolerance
    }

    /**
     * Returns the current setpoint of the PIDFController.
     *
     * @return The current setpoint.
     */
    fun getSetPoint(): Double = setPoint

    /**
     * Sets the setpoint for the PIDFController
     *
     * @param sp The desired setpoint.
     */
    fun setSetPoint(sp: Double) {
        setPoint = sp
        positionError = setPoint - measuredValue
        velocityError = (positionError - prevErrorVal) / period
    }

    /**
     * Returns true if the error is within the percentage of the total input range, determined by
     * [.setTolerance].
     *
     * @return Whether the error is within the acceptable bounds.
     */
    fun atSetPoint(): Boolean =
        abs(positionError) < errorTolerance_p &&
            abs(velocityError) < errorTolerance_v

    val coefficients: DoubleArray
        /**
         * @return the PIDF coefficients
         */
        get() = doubleArrayOf(p, i, d, f)

    val tolerance: DoubleArray
        /**
         * @return the tolerances of the controller
         */
        get() = doubleArrayOf(errorTolerance_p, errorTolerance_v)

    /**
     * Calculates the next output of the PIDF controller.
     *
     * @param pv The given measured value.
     * @param sp The given setpoint.
     * @return the next output using the given measurd value via
     * [.calculate].
     */
    fun calculate(
        pv: Double,
        sp: Double,
    ): Double {
        // set the setpoint to the provided value
        setSetPoint(sp)
        return calculate(pv)
    }

    /**
     * Calculates the control value, u(t).
     *
     * @param measuredValue The current measurement of the process variable.
     * @return the value produced by u(t).
     */

    /**
     * Calculates the next output of the PIDF controller.
     *
     * @return the next output using the current measured value via
     * [.calculate].
     */
    @JvmOverloads
    fun calculate(pv: Double = measuredValue): Double {
        prevErrorVal = positionError

        val currentTimeStamp = System.nanoTime().toDouble() / 1E9
        if (lastTimeStamp == 0.0) lastTimeStamp = currentTimeStamp
        period = currentTimeStamp - lastTimeStamp
        lastTimeStamp = currentTimeStamp

        if (measuredValue == pv) {
            positionError = setPoint - measuredValue
        } else {
            positionError = setPoint - pv
            measuredValue = pv
        }

        if (abs(period) > 1E-6) {
            velocityError = (positionError - prevErrorVal) / period
        } else {
            velocityError = 0.0
        }

        /*
        if total error is the integral from 0 to t of e(t')dt', and
        e(t) = sp - pv, then the total error, E(t), equals sp*t - pv*t.
         */
        totalError += period * (setPoint - measuredValue)
        totalError = if (totalError < minIntegral) minIntegral else min(maxIntegral, totalError)

        // returns u(t)
        return sqrt(p * abs(positionError)) * sign(positionError) + i * totalError + d * velocityError + f * setPoint
    }

    fun setPIDF(
        kp: Double,
        ki: Double,
        kd: Double,
        kf: Double,
    ) {
        p = kp
        i = ki
        d = kd
        f = kf
    }

    fun setIntegrationBounds(
        integralMin: Double,
        integralMax: Double,
    ) {
        minIntegral = integralMin
        maxIntegral = integralMax
    }

    fun clearTotalError() {
        totalError = 0.0
    }
}

class SQUIDController(
    kp: Double,
    ki: Double,
    kd: Double,
) : SQUIDFController(kp, ki, kd, 0.0) {
    fun setPID(
        kp: Double,
        ki: Double,
        kd: Double,
    ) {
        setPIDF(kp, ki, kd, 0.0)
    }
}
