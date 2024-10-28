package com.millburnx.pathPlanner.layout

import com.millburnx.util.geometry.Vec2dI
import java.awt.event.ComponentAdapter
import javax.swing.JFrame
import javax.swing.JLayeredPane

public class StackLayout : JLayeredPane() {
    public val popoverLayer: PopoverLayer = PopoverLayer()
    public val mainLayout: MainLayout = MainLayout()

    init {
        add(mainLayout, DEFAULT_LAYER, 1)
        add(popoverLayer, POPUP_LAYER, 0)
    }

    public fun addTo(frame: JFrame) {
        frame.add(this)
        frame.addComponentListener(object : ComponentAdapter() {
            override fun componentResized(e: java.awt.event.ComponentEvent) {
                updateBounds(Vec2dI(frame.width, frame.height))
            }
        })
        updateBounds(Vec2dI(frame.width, frame.height))
    }

    public fun updateBounds(frameSize: Vec2dI) {
        setBounds(0, 0, frameSize.x, frameSize.y)
        components.forEach { it.setBounds(0, 0, width, height); it.revalidate() }
        revalidate()
        repaint()
    }
}