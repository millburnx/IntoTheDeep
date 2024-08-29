package com.millburnx.pathplanner.Layout

import javax.swing.JFrame
import javax.swing.JLayeredPane

class StackLayout : JLayeredPane() {
    val popoverLayer = PopoverLayer()
    val mainLayout = MainLayout()

    init {
        add(mainLayout, DEFAULT_LAYER, 1)
        add(popoverLayer, POPUP_LAYER, 0)
    }

    fun addTo(frame: JFrame) {
        frame.add(this)
        setBounds(0, 0, frame.width, frame.height)
        components.forEach { it.setBounds(0, 0, width, height) }
    }
}

