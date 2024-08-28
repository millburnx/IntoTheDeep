package com.millburnx.dashboard.canvas.canvasOp

import com.millburnx.dashboard.canvas.FTCDashboardImpl
import com.millburnx.utils.Scale

class Scale(private val scaleX: Double, private val scaleY: Double) : CanvasOpImpl(Type.SCALE) {
    override fun draw(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale) {
        ctx.g2d.scale(scaleX, scaleY)
    }
}