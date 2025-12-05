# SICKSense Agripollinate Pi Program

This directory contains the integrated master program that combines LiDAR object detection with automated image capture.

## Files

### Core Modules
- **`master_main.py`** - Main program coordinating all subsystems
- **`lidar_parser.py`** - LiDAR connection and data parsing module
- **`object_detector.py`** - Real-time object detection and tracking module
- **`image_capture.py`** - Image capture and transmission module

### Client Program
- **`client_main.py`** - Image client (run on separate device)

## Architecture

The master program maintains two simultaneous TCP connections:
1. **LiDAR Connection** - Continuous stream from SICK TiM561 scanner
2. **Image Server** - Sends captured images to remote client

### Data Flow
```
LiDAR Scanner → lidar_parser → object_detector → [TRIGGER] → image_capture → Image Client
     (TCP)                     (real-time)                      (burst)         (TCP)
```

## Configuration

Edit parameters in `master_main.py`:

### LiDAR Settings
- `LIDAR_HOST` - LiDAR IP address (default: 192.168.137.2)
- `LIDAR_PORT` - LiDAR port (default: 2112)

### Image Server Settings
- `IMAGE_SERVER_HOST` - Server bind address (default: 0.0.0.0)
- `IMAGE_SERVER_PORT` - Server port (default: 12345)
- `IMAGE_SAVE_DIR` - Directory to save images
- `BURST_SIZE` - Number of images per trigger (default: 5)

### Detection Parameters
- `EPS` - DBSCAN cluster distance (default: 0.25m)
- `MIN_SAMPLES` - Minimum points per cluster (default: 3)
- `NEW_OBJ_DIST` - Distance threshold for new objects (default: 0.3m)
- `OBJECT_MEMORY_TIME` - Memory duration (default: 1.0s)
- `MIN_DETECTIONS` - Scans before trigger (default: 3)
- `TRIGGER_COOLDOWN` - Minimum time between triggers (default: 2.0s)
- `ESTABLISHED_THRESHOLD` - Time to be "established" (default: 2.0s)

### Watchdog
- `WATCHDOG_TIMEOUT` - Maximum time between scans (default: 5.0s)

## Usage

### On Raspberry Pi (Server)
```bash
cd pi_program
python3 master_main.py
```

The program will:
1. Connect to LiDAR scanner
2. Start image server and wait for client
3. Begin real-time object detection
4. Capture image bursts on triggers

### On Remote Device (Client)
```bash
cd pi_program
python3 client_main.py
```

Update `HOST` in `client_main.py` to match your Raspberry Pi's IP address.

## Features

- **Dual TCP Connections** - Simultaneous LiDAR and image client connections
- **Real-Time Processing** - No file I/O, direct stream processing
- **Object Tracking** - Three categories: new, tracking, established
- **Trigger Cooldown** - Prevents rapid-fire triggers (2-second minimum)
- **Watchdog Timer** - Monitors LiDAR data stream health
- **Graceful Shutdown** - Ctrl+C handler for clean exit
- **Automatic Reconnection** - Attempts to reconnect on LiDAR timeout

## Dependencies

```bash
pip install numpy scikit-learn
```

System requirements:
- `fswebcam` for image capture (install: `sudo apt-get install fswebcam`)

## Notes

- The master program must connect to both LiDAR and image client before starting detection
- Image client should be started before or shortly after the master program
- All original files remain unchanged in their respective directories
- Statistics are printed every 100 scans and on shutdown
