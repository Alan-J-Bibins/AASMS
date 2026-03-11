# Autonomous Altitude and Safety Management System for Hot Air Balloons

#### An embedded systems project by Sukruta Nadkarni and Alan J Bibins

## Components
The system is built around the Arduino Nano/Uno and integrates the following components:
- Altitude/Temp: BMP280/MS5611 Barometer.
- Positioning: Ublox NEO-6M GPS.
- Safety/IMU: MPU6050/BNO055 Inertial Sensor for tilt and turbulence.
- Communication: SX1278 LoRa for long-range telemetry.
- Control: MG996R Servo for burner valve regulation.

## Repo structure

```
.
├── diagram.json        : The wokwi visualization file, whatever you do in the online editor generates this json file, so yea use this with the web app for designiing the circuit
├── include             : Header files associated with the project go here
│   └── README
├── lib                 : So some components' libraries may be private, this folder will contain those specific libraries, pio will compile them to static libraries and link it to the executable file
│   └── README
├── platformio.ini      : Config file for pio
├── README.md           : This readme that you are reading
├── src                 : This is where most of our code will go
│   └── main.cpp
├── test                : Tests and shit go here
│   └── README
└── wokwi.toml          : wokwi-cli config file
```


## Project setup

We are using platformio for setting up and running the codebase, with wokwi to make the simulate the circuit

### wokwi-cli

We are going to use this run our code without the components (coz we don't have them rn)

Windows:
```powershell
iwr https://wokwi.com/ci/install.ps1 -useb | iex
```

To run:
```powershell
wokwi-cli
```
You may be prompted to get a CI Token, go ahead and make that

### platformio-core (The cli for PlatformIO)

> [!NOTE]
> There is a vscode extension for platformio, I'd suggest using it. Perhaps if you use the extension you may not need the cli so try it out first before doing the following.
>
> Windows:
```powershell
wget -O get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
```

> ## Reference links:

### BMP280
- https://wokwi.com/projects/431553492560956417

### MPU6050 
- https://wokwi.com/projects/305936654686749250
