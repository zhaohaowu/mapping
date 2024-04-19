# Localization

## 1. Introduction

​	This module provides localization services for cars, which is targeted to low cost but high precision. Localization module is separated into front and back end, the front end mainly provides an initial localization results for the car with high frequency, then the back end rectify the front end localization drift according to off-line map.

​	Now, this module supports both on-line and off-line localization from multi-sensor: 

- [x] GNSS - The Global Navigation Satellite System
- [x] IMU - Inertial Measurement Unit
- [x] CAN - Controller Area Network, actually car odometer
- [x] Camera - for Visual Odometry or MapMatching

The front end now provides localization methods as follow:

```
1. INS, integrated navigation results from Novatel or OXTS device
2. CAN, dead reckoning from car odometer
3. IMU, dead reckoning from IMU
...
```

The back end provides localization method :
```
1. SMM, semantic map matching, visual vehicle localization based on traffic semantic segmantation(dash line, solid line, road arrow, etc) and semantic map.
2. GNSS, source from GPS、BDS、GLONASS and so on.
...
```