# MPU6050_light

**Lightweight, fast and simple library to communicate with the MPU6050**

:arrow_down: The source code is available at [https://github.com/lyubomyrk/MPU6050_light_pico](https://github.com/lyubomyrk/MPU6050_light_pico)

:arrows_counterclockwise: Your feedback is important. Any issue or suggestion can be reported to the Github `Issues` section


## Description

The library is made to retrieve accelerometer and gyroscope measurements from the MPU6050. This data is processed using a complementary filter to provide and estimation of tilt angles on X and Y with respect to the horizontal frame. The hypothesis for the validity of these angles are:
* small linear accelerations (the gravity is the dominant one)
* small loop delay between two calls to `update()` so the approximation `angle[t]=angle[t-1]+gyro*dt` is valid
* heading (angle Z) is valid for small X and Y angles

## Documentation

A documentation PDF is provided within the library folder, otherwise get it online at [https://github.com/rfetick/MPU6050_light](https://github.com/rfetick/MPU6050_light). It includes definitions of the functions and gives a minimal example of usage of the code.

Additionaly, look into the main.cpp file for an example of how to use the library.

## License

See the LICENSE file

## Authors

[lyubomyrk](https://github.com/lyubomyrk) : ported to pico using the SDK.

[rfetick](https://github.com/rfetick) : modifications for better memory management, speed and efficiency.

[tockn](https://github.com/tockn) : initial author of the library (v1.5.2)

The library has also been improved thanks to the comments of [edgar-bonet](https://github.com/edgar-bonet) and [augustosc](https://github.com/augustosc)
