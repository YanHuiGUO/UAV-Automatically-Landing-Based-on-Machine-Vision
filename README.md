# UAV-Automatically-Landing-Based-on-Machine-Vision
* This is a source code of UAV-Automatically-Landing-Based-on-Machine-Vision using C language based on STM32F4 MCU, USB Camera and Mini Inter NUC. This project is based on C++.
      * Recognize_Target - Running in Inter NUC
         * Recognize_Target\src - including the image process codes
         * Building - Using VS2015
      * Control - Based on the open source flight control software APM
         * UserCode.pde - The main navigation control and data tranformation code
         * Building - Open ArduCopter.sln Using VS2015
      * Middleware - Running in STM32F4 MCU
         * \USER - Inlcuding the codes tranform data from Inter NUC to APM 
         * Building - Open Analysis_Board.uvoptx Using Keil Uvision5
         * Codes also support MarvLink, you can receive and deal your information using ROS
      * Middleware(PCB) - Including the Schematic and PCB file of the circuit
