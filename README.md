# FreeRTOS-Based Embedded Menu System on STM32

This project implements a **FreeRTOS-based interactive menu system** on an STM32F4 microcontroller. It allows users to control LED effects and view or set the RTC (Real-Time Clock) time and date via UART commands.

---

## 📅 Project Timeline

- **Developed:** September 2024  
- **Published on GitHub:** May 2025  
- **Platform:** STM32F439ZI 
- **RTOS:** FreeRTOS  
- **Toolchain:** STM32CubeIDE / STM32 HAL / SEGGER SystemView 

---

## 📦 Features

- 🕹️ UART-based interactive CLI (menu-driven)
- 🔁 LED effect selection (`e1`, `e2`, `e3`)
- 🕒 View and set RTC time and date
- ✅ Input validation with error prompts
- 🧵 Modular FreeRTOS tasks (`MENU`, `PRINT`, `CMD`, `LED`, `RTC`)
- 📨 Queue + Notification based inter-task communication

---

## 🧪 Usage

### UART Configuration:
- **Baud Rate:** 115200
- **Word Length:** 8 bits
- **Stop Bits:** 1
- **Parity:** None
- **Flow Control:** None

### Command Menu:

\!\!\!Welcome to our Show\!\!\!
Select one from below:
1 -> LED_SHOW
2 -> TIME_SHOW

- Press `1`: Control LEDs with commands `e1`, `e2`, `e3`, `none`
- Press `2`: Access RTC menu
  - `o1`: Show current time
  - `o2`: Set time (via prompts for sec/min/hours)
  - `o3`: (Optional for future: set date)
- Use `#` to return to main menu

---

## 🧰 Dependencies

- STM32 HAL
- FreeRTOS
- SEGGER SystemView / RTT (for debugging)

---

## 🛠️ Build & Flash

1. Open project in **STM32CubeIDE**
2. Connect your STM32 board via ST-Link
3. Build the project
4. Flash to target (`Run` → `Debug As` → `STM32 Cortex-M C/C++ Application`)

---

## 🚀 Future Work

- Add date-setting functionality
- Non-blocking UART with ring buffer
- Persistent RTC via backup registers
- Power-saving integration

---

## 📝 License

Parts of this project are based on STM32CubeMX-generated code, which is © STMicroelectronics and provided under their original licensing terms.  
All additional application code is shared **as-is**, with no warranties.

---

## 🤝 Acknowledgements

Developed as part of embedded systems learning — particularly to practice:
- RTOS task structuring
- Inter-task communication
- Peripheral initialization (RTC, GPIO, UART)

## Note
Note: Only `Core/Src` is included in this repository to keep the project focused on application logic.  
To build the project:
- Use STM32CubeIDE
- Add STM32 HAL drivers (via CubeMX or manually)
- Include FreeRTOS and SEGGER (optional for debugging)
