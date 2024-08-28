package com.millburnx.dashboard.canvas.canvasOp

import com.millburnx.dashboard.canvas.FTCDashboardImpl
import com.millburnx.utils.Scale

class Rotation(private val rotation: Double) : CanvasOpImpl(Type.ROTATION) {
    override fun draw(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale) {
        ctx.g2d.rotate(rotation)
    }
}