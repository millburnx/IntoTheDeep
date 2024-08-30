package com.millburnx.pathplanner.ui.layout

import javax.swing.JButton
import javax.swing.JPanel
import javax.swing.border.EmptyBorder

class Navbar : JPanel() {
    val save = JButton("Save").apply {
    }
    val load = JButton("Load").apply {
    }
    val clear = JButton("Clear").apply {
    }

    init {
        layout = WrapLayout()
        border = EmptyBorder(16, 24, 16, 24)
        add(save)
        add(load)
        add(clear)
    }
}