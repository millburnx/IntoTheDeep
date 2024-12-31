package com.millburnx.utils

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.boolean
import kotlinx.serialization.json.double
import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long
import java.io.File

class ConfigLoader(file: File) {
    val json = Json.parseToJsonElement(file.readText()).jsonObject

    inline fun <reified T> get(key: String, default: T): T {
        val element = json[key]
        return try {
            when {
                element == null -> default
                T::class == Boolean::class -> element.jsonPrimitive.boolean as T
                T::class == Int::class -> element.jsonPrimitive.int as T
                T::class == Long::class -> element.jsonPrimitive.long as T
                T::class == Double::class -> element.jsonPrimitive.double as T
                T::class == String::class -> element.jsonPrimitive.content as T
                else -> throw IllegalArgumentException("Unsupported type")
            }
        } catch (e: Exception) {
            default
        }
    }
}

class ConfigLoaderTest {
    companion object {
        val config = ConfigLoader(File("configs/config.json"))

        @JvmStatic
        var specimens: Boolean = config.get<Boolean>("specimens", false)

        @JvmStatic
        var specimenCount: Int = config.get<Int>("specimenCount", 0)

        @JvmStatic
        var humanPlayerOffset: Double = config.get<Double>("humanPlayerOffset", 0.0)
    }
}

fun main() {
    println("ConfigLoaderTest.config = ${ConfigLoaderTest.config.json}")
    println("ConfigLoaderTest.specimens = ${ConfigLoaderTest.specimens}")
    println("ConfigLoaderTest.specimenCount = ${ConfigLoaderTest.specimenCount}")
    println("ConfigLoaderTest.humanPlayerOffset = ${ConfigLoaderTest.humanPlayerOffset}")
}