Team 7652
[![Build & Format Check](https://github.com/MiamiBeachBots/base-bot/actions/workflows/gradle-build.yml/badge.svg)](https://github.com/MiamiBeachBots/base-bot/actions/workflows/gradle-build.yml)

Install WPILib

The official WPILib installer is the best way to get started. It handles setting up all the tools you need, including Visual Studio Code and the Java Development Kit (JDK).

    Windows: Download and run the .iso installer from the WPILib website. Mount the disk image, then launch WPILibInstaller.exe.

    macOS: Download the .dmg installer and run the WPILibInstaller application.

    Linux: Download the .tar.gz file. Extract it and run the installer script from the terminal.

Build

To compile your code, you can use the built-in tools in Visual Studio Code or the command line.

    VS Code: Open the Command Palette (Ctrl + Shift + P) and select WPILib: Build Robot Code.

    Command Line: Open a terminal in the project's root directory and run the Gradle Wrapper script.

        macOS/Linux: ./gradlew build

        Windows: gradlew.bat build

Update code on robot

To get your new code onto the RoboRIO, you need to "deploy" it. Make sure your computer is connected to the robot via USB, Ethernet, or Wi-Fi.

    VS Code: Use the Command Palette (Ctrl + Shift + P) and select WPILib: Deploy Robot Code.

    Command Line: In your terminal, run the deploy command:

        macOS/Linux: ./gradlew deploy

        Windows: gradlew.bat deploy

TODO

    [x] Finish Gyro

    [x] Pathweaver/trajectory planning

    [x] Network Tables

    [x] Py Coprocessor

    [x] Basic Dashboard

    [ ] Fancy Dashboard

    [ ] Finish the readme

    [x] CAN support
