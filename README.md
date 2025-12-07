# Code Organization

### File Structure
```
Embedded/
├── main/
│   ├── main.c              (648 lines - all functionality)
│   └── CMakeLists.txt
├── build/                  (Generated build artifacts)
├── CMakeLists.txt          (Project configuration)
├── sdkconfig               (ESP-IDF configuration)
---
# Build Instructions:

```bash
# Set up ESP-IDF environment
. $HOME/esp/esp-idf/export.sh  # Linux/Mac
# OR
%USERPROFILE%\esp\esp-idf\export.bat  # Windows

# Configure WiFi credentials in main.c
# Edit WIFI_SSID and WIFI_PASS

# Build project
cd Embedded
idf.py build

# Flash to ESP32
idf.py -p COM9 flash  # Windows
idf.py -p /dev/ttyUSB0 flash  # Linux

# Monitor serial output
idf.py -p COM9 monitor

# Access web dashboard
# Check serial output for IP address
# Open browser: http://<ESP32_IP_ADDRESS>
```
---
