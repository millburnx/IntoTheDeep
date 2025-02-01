apply("../build.common.gradle")
apply("../build.dependencies.gradle")

android {
    namespace = "com.millburnx.jsoncommands"
    compileSdk = 34

    defaultConfig {
        minSdk = 21
        targetSdk = 34
    }

    packagingOptions {
        jniLibs {
            pickFirsts.add("**/*.so")
        }
        jniLibs.useLegacyPackaging = true
    }

    kotlinOptions {
        jvmTarget = "11"
    }
}

plugins {
    id("com.android.application")
    kotlin("android")
//    kotlin("jvm")
}

kotlin {
    explicitApi()
}

group = "com.millburnx.jsoncommands"
version = "0.1.0"

repositories {
    mavenCentral()
}

dependencies {
//    testImplementation(kotlin("test"))
    implementation(kotlin("reflect"))
    implementation("com.google.code.gson:gson:2.12.1")
//    implementation("androidx.core:core-ktx:1.13.1")
    implementation("org.ftclib.ftclib:core:2.1.1") {
        exclude("org.firstinspires.ftc")
        exclude("androidx.core")
    }
}

// tasks.test {
//    useJUnitPlatform()
// }
