# ECEN_403_SICK_TEAM2

## Agripollinate Single Board Computer (SBC) Subsystem

This repository contains the software for the Agripollinate project's Single Board Computer subsystem, which integrates LiDAR-based object detection with automated image capture for agricultural pollination monitoring.

## Directory Structure

### `imagescripts/`
Contains preliminary versions of the image capture and transmission functionality. These scripts demonstrate the basic client-server architecture for capturing and sending images over TCP.

### `LiDARscripts/`
Contains preliminary and separate versions of the LiDAR data logging and object detection functionality. These scripts demonstrate LiDAR data parsing, clustering algorithms, and object tracking independently.

### `pi_program/` 
**Contains the final integrated program** that combines all functionalities into a cohesive system designed to run on the Raspberry Pi Single Board Computer. This is the final version of the subsystem at the end of ECEN 403.

**See [`pi_program/README.md`](pi_program/README.md) for detailed documentation, configuration, and usage instructions.**

## Development Approach

The preliminary scripts in `imagescripts/` and `LiDARscripts/` were developed and tested independently to validate each feature before integration. The `pi_program/` directory represents the final synthesis of these components into a unified real-time system.