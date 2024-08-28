package com.millburnx.dashboard.canvas.canvasOp

import com.millburnx.dashboard.canvas.FTCDashboardImpl
import com.millburnx.utils.Scale

class Alpha(val alpha: Double) : CanvasOpImpl(Type.ALPHA) {
    override fun draw(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale) {
        ctx.alpha = alpha
    }
}