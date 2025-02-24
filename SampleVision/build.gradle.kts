apply("../build.common.gradle")
apply("../build.dependencies.gradle")

android {
    namespace = "com.millburnx.samplevision"
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

group = "com.millburnx.samplevision"
version = "0.1.0"

repositories {
    mavenCentral()
}

dependencies {
    implementation("org.ftclib.ftclib:core:2.1.1")
    implementation(project(":PathPlanner"))
}
