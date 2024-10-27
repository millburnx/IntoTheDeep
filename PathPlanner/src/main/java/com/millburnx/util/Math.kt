package com.millburnx.util

import kotlin.math.round

public object Math {
    public fun lerp(a: Double, b: Double, t: Double): Double {
        return a + (b - a) * t
    }

    public fun lerp(a: Float, b: Float, t: Double): Float {
        return a + (b - a) * t.toFloat()
    }

    public fun lerp(a: Int, b: Int, t: Double): Int {
        return round(a + (b - a) * t).toInt()
    }
}
