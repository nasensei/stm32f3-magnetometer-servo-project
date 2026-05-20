### README
# MTRX2700 Assignment 2 — Programming a Microcontroller in C

## Project Overview

This repository contains our group solution for **MTRX2700 Mechatronics 2 – Lab 2: Programming a Microcontroller in C**.  
The project focuses on designing reusable C modules for the STM32 microcontroller and integrating them into a complete embedded system.

Across this assignment, we developed and tested software modules for:

- Digital input/output
- Timer-based callbacks and PWM generation
- UART serial communication
- I2C sensor interfacing
- Integration of all modules into a multi-board application

The overall aim of this project is not only to produce working code, but also to demonstrate good embedded software design through:

- modular structure
- encapsulation of complexity
- clearly defined interfaces
- reuse and extendability
- testing and validation of each module

---

## Team Members and Responsibilities

| Name | SID | Primary Responsibility | Secondary Responsibility |
|------|-----|------------------------|--------------------------|
| `Selina Nguyen` | `530531201` | `UART serial module` | `Integration` |
| `Tung Lin Wu` | `550718161` | `PWM timer section` | `Integration` |
| `<Member 3>` | `<SID>` | `<e.g. Digital I/O + LED/Button module>` | `<Testing / debugging>` |
| `<Member 4>` | `<SID>` | `<e.g. I2C magnetometer + integration>` | `<Minutes / documentation>` |

### Contribution Summary
Each team member took lead responsibility for one or more modules, while all members were expected to understand the design, implementation, and testing of the full system. The repository history, meeting minutes, and code contributions together document how the work was planned and distributed.

---

## Assignment Objectives

This project demonstrates how to use the C language on the STM32 platform to:

- work with memory and pointers in embedded systems
- use interrupts for asynchronous communication
- use timers for time-critical tasks and PWM generation
- communicate with sensors over I2C
- integrate multiple software modules into a working embedded application

---

## Repository Structure

```text
MTRX2700-assignment-2/
├── 7.1_Digital_IO/
│   ├── <CubeIDE project files / source / headers>
│   └── <exercise-specific documentation or notes>
├── 7.2_Timer_Interface/
│   ├── <CubeIDE project files / source / headers>
│   └── <exercise-specific documentation or notes>
├── 7.3_Serial_Interface/
│   ├── <CubeIDE project files / source / headers>
│   └── <exercise-specific documentation or notes>
├── 7.4_I2C_Sensor_Interfacing/
│   ├── <CubeIDE project files / source / headers>
│   └── <exercise-specific documentation or notes>
├── 7.5_Integration_Task/
│   ├── <CubeIDE project files / source / headers>
│   └── <exercise-specific documentation or notes>
├── minutes/
│   ├── <meeting agenda/minutes files>
│   └── <planning notes>
└── README.md
=======
# stm32f3-magnetometer-servo-project
>>>>>>> 4f3b72716a894e8f8477a75286b01d316c8975dd
