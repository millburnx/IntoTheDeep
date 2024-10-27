package com.millburnx.util.curves

import com.millburnx.util.IVec2d

public interface Curve {
    public val points: List<IVec2d>

    public fun at(t: Double): IVec2d
}