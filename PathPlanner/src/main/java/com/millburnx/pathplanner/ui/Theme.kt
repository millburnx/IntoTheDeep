package com.millburnx.pathplanner.ui

import com.formdev.flatlaf.intellijthemes.FlatOneDarkIJTheme
import java.awt.Color

class Theme {
    companion object {
        var scale: Double = 1.0
        
        val LAF = FlatOneDarkIJTheme()
        val bg0 = Color(0x21252b)
        val bg1 = Color(0x282c34)
        val fg = Color(0xabb2bf)
        val borderColor = Color(0x333841)
        val primary = Color(0x568AF2)
    }

    class Accents {
        companion object {
            val red = Color(0xe06c75)
            val green = Color(0x98c379)
            val yellow = Color(0xe5c07b)
            val blue = Color(0x61afef)

            val purple = Color(0xc678dd)
            val cyan = Color(0x56b6c2)
            val orange = Color(0xd19a66)
        }
    }
}