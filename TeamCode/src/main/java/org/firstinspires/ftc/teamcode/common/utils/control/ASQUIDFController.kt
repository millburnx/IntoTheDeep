package org.firstinspires.ftc.teamcode.common.utils.control

import org.firstinspires.ftc.teamcode.common.utils.normalizeDegrees
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sqrt

/**
 * This is a PID controller (https://en.wikipedia.org/wiki/PID_controller)
 * for your robot. Internally, it performs all the calculations for you.
 * You need to tune your values to the appropriate amounts in order
 * to properly utilize these calculations.
 *
 *
 * The equation we will use is:
 * u(t) = kP * e(t) + kI * int(0,t)[e(t')dt'] + kD * e'(t) + kF
 * where e(t) = r(t) - y(t) and r(t) is the setpoint and y(t) is the
 * measured value. If we consider e(t) the positional error, then
 * int(0,t)[e(t')dt'] is the total error and e'(t) is the velocity error.
 */
open class ASQUIDFController
    @JvmOverloads
    constructor(
        kp: Double,
        ki: Double,
        kd: Double,
        kf: Double,
        sp: Double = 0.0,
        pv: Double = 0.0,
    ) {
        private var kP: Double
        private var kI: Double
        private var kD: Double
        private var kF: Double
        private var setPoint: Double
        private var measuredValue: Double
        private var minIntegral: Double
        private var maxIntegral: Double

        private var errorValP: Double
        private var errorValV = 0.0

        private var totalError = 0.0
        private var prevErrorVal = 0.0

        private var errorToleranceP = 0.05
        private var errorToleranceV = Double.Companion.POSITIVE_INFINITY

        private var lastTimeStamp: Double
        private var period: Double

        /**
         * This is the full constructor for the PIDF controller. Our PIDF controller
         * includes a feed-forward value which is useful for fighting friction and gravity.
         * Our errorVal represents the return of e(t) and prevErrorVal is the previous error.
         *
         * @param sp The setpoint of the pid control loop.
         * @param pv The measured value of he pid control loop. We want sp = pv, or to the degree
         * such that sp - pv, or e(t) < tolerance.
         */

        /**
         * The base constructor for the PIDF controller
         */
        init {
            kP = kp
            kI = ki
            kD = kd
            kF = kf

            setPoint = sp
            measuredValue = pv

            minIntegral = -1.0
            maxIntegral = 1.0

            lastTimeStamp = 0.0
            period = 0.0

            errorValP = setPoint - measuredValue
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
            setTolerance(positionTolerance, Double.Companion.POSITIVE_INFINITY)
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
            errorToleranceP = positionTolerance
            errorToleranceV = velocityTolerance
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
            errorValP = setPoint - measuredValue
            errorValV = (errorValP - prevErrorVal) / period
        }

        /**
         * Returns true if the error is within the percentage of the total input range, determined by
         * [.setTolerance].
         *
         * @return Whether the error is within the acceptable bounds.
         */
        fun atSetPoint(): Boolean =
            abs(errorValP) < errorToleranceP &&
                abs(errorValV) < errorToleranceV

        /**
         * @return the PIDF coefficients
         */
        fun getCoefficients(): DoubleArray = doubleArrayOf(kP, kI, kD, kF)

        /**
         * @return the positional error e(t)
         */
        fun getPositionError(): Double = errorValP

        /**
         * @return the tolerances of the controller
         */
        fun getTolerance(): DoubleArray = doubleArrayOf(errorToleranceP, errorToleranceV)

        /**
         * @return the velocity error e'(t)
         */
        fun getVelocityError(): Double = errorValV

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
         * @param pv The current measurement of the process variable.
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
            prevErrorVal = errorValP

            val currentTimeStamp = System.nanoTime().toDouble() / 1E9
            if (lastTimeStamp == 0.0) lastTimeStamp = currentTimeStamp
            period = currentTimeStamp - lastTimeStamp
            lastTimeStamp = currentTimeStamp

            if (measuredValue == pv) {
                errorValP = normalizeDegrees(setPoint - measuredValue)
            } else {
                errorValP = normalizeDegrees(setPoint - pv)
                measuredValue = pv
            }

            if (abs(period) > 1E-6) {
                errorValV = (errorValP - prevErrorVal) / period
            } else {
                errorValV = 0.0
            }

        /*
        if total error is the integral from 0 to t of e(t')dt', and
        e(t) = sp - pv, then the total error, E(t), equals sp*t - pv*t.
         */
            totalError += period * (setPoint - measuredValue)
            totalError = if (totalError < minIntegral) minIntegral else min(maxIntegral, totalError)

            // returns u(t)
            return sqrt(kP * abs(errorValP)) * sign(errorValP) + kI * totalError + kD * errorValV + kF * setPoint
        }

        fun setPIDF(
            kp: Double,
            ki: Double,
            kd: Double,
            kf: Double,
        ) {
            kP = kp
            kI = ki
            kD = kd
            kF = kf
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

        fun setP(kp: Double) {
            kP = kp
        }

        fun setI(ki: Double) {
            kI = ki
        }

        fun setD(kd: Double) {
            kD = kd
        }

        fun setF(kf: Double) {
            kF = kf
        }

        fun getP(): Double = kP

        fun getI(): Double = kI

        fun getD(): Double = kD

        fun getF(): Double = kF

        fun getPeriod(): Double = period
    }

class ASQUIDController(
    kp: Double,
    ki: Double,
    kd: Double,
) : ASQUIDFController(kp, ki, kd, 0.0) {
    fun setPID(
        kp: Double,
        ki: Double,
        kd: Double,
    ) {
        setPIDF(kp, ki, kd, 0.0)
    }
}
