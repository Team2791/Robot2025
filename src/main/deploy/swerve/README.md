All constants in this directory are taken from design. Since I
cannot add comments in JSON, I'm going to document them all here.

You're welcome,
<br /> Angad.

# Swerve Drive Configuration

- Everything is in freedom units (inches, pounds, etc.)

## `servedrive.json`

### [**Online Documentation**](https://docs.yagsl.com/configuring-yagsl/configuration/serve-drive-configuration)

- `imu` [AKA: gyroscope] [[YAGSL](https://docs.yagsl.com/devices/gyroscope#gyroscope-configuration)]
    - `type`: The gyroscope type. We currently use "navx," connected through MXP SPI, so `navx_spi` is used.
    - `id`: Set to `0`, since the NavX doesn't use an ID.
    - `canbus`: Set to `null`, since the NavX doesn't use the CAN bus.
- `invertedIMU`: Set to `true` iff the NavX is mounted upside down.
- `modules`: An array of `json` files in the `modules/` directory defining the serve modules.

## `modules/physicalproperties.json`

### [**Online Documentation**](https://docs.yagsl.com/configuring-yagsl/configuration/physical-properties-configuration)

- `conversionFactors`
    - `angle`: Turn motor properties
        - `gearRatio`: The gear ratio of the turn motor to the wheel. See Matt Stormer
    - `drive`: Drive motor properties
        - `gearRatio`: The gear ratio of the drive motor to the wheel. Calculated using the following:
          ```java
          // Number of teeth on the pinion gear. According to docs, either 12, 13, or 14T. See Matt Stormer
          double kPinionTeeth = 14.0;
    
          // Number of teeth on the wheel's bevel gear
          double kBevelGearTeeth = 45.0;
    
          // Number of teeth on the first-stage spur gear
          double kSpurTeeth = 22.0;
    
          // Number of teeth on the bevel pinion
          double kBevelPinionTeeth = 15.0;
    
          // Reduction of input(motor) to output(wheel).
          double kRatio = (kBevelGearTeeth * kSpurTeeth) / (kPinionTeeth * kBevelPinionTeeth);
          ```
        - `diameter`: The diameter of the wheel in inches
- `currentLimit` (in Amps):
    - `angle`: The current limit for the turn motor. See Chris Lane
    - `drive`: The current limit for the drive motor. See Chris Lane
- `robotMass`: The mass of the robot in pounds. See Ross
- `steerRotationalInertia`: The rotational inertia of the steer mechanism in **Kilogram square meters**. See Ross
- `wheelGripCoefficientOfFriction`: The coefficient of friction of our _custom_ wheels on the carpet. See Ross
- `optimalVoltage` [AKA: nominal voltage]: Ask Chris Lane what this means. I was told 12V last year.

### To implement!

- `rampRate`: The number of seconds it takes for the motor to go from 0 to full throttle. Test this in REV Hardware
  Client.
  ```json
  "rampRate": {
    "drive": 123.456,
    "angle": 123.456
  },
  ```

## `modules/pidfproperties.json`

- No docs here, all is explained on YAGSL
- [**READ THIS FIRST**](https://docs.yagsl.com/configuring-yagsl/how-to-tune-pidf)
- [Syntax Docs](https://docs.yagsl.com/configuring-yagsl/configuration/pidf-properties-configuration)

## `modules/{module}.json`

### [**Online Documentation**](https://docs.yagsl.com/configuring-yagsl/configuration/swerve-module-configuration)

**NOTE**: **B**attery is in the **B**ack. Left and right are from the top-down perspective.

- `drive`: Drive motor properties
    - `type`: Type of the motor. Our motors are `neo` brushless motors.
    - `id`: The CAN ID of the motor controller. Talk to controls about this.
        - **NOTE:** For the 2026 and onwards robots, try to standardize this to
            - Front Left: 10
            - Front Right: 20
            - Back Right: 30
            - Back Left: 40
            - i.e. Clockwise starting from the front left, in tens.
    - `canbus`: Set to `null`, Rev motors don't support `canivore`
- `angle`: Turn motor properties
    - `type`: The type of the motor. Our motors are `neo550`s.
    - `id`: Same as the drive motor, plus five
    - `canbus`: Still `null`
- `encoder`: Absolute encoder properties
    - `type`: The type of the encoder. We use Rev's `attached` absolute encoders.
    - `id`: Set to `0`, since encoders are attached to the motor controllers
    - `canbus`: Still `null`
- `inverted`: These are for the motors, set both to false unless we have motor problems (2024 off-season)
- `absoluteEncoderInverted`: Set to `true` due to SparkSwerve quirks
- `absoluteEncoderOffset`: The offset of the absolute encoder in degrees. Use the brackets, and convert encoder outputs.
  from radians to degrees.
- `location`: The location of the module on the robot. To calculate:
    - let $x$ be the distance, in inches, between the centers of the front and back wheels[^1]
    - let $y$ be the distance, in inches, between the centers of the left and right wheels
    - `location.front` is $\frac{x}{2}$ if we are a "front" module, or $-\frac{x}{2}$ if we are a "back" module
    - `location.left` is $\frac{y}{2}$ if we are a "left" module, or $-\frac{y}{2}$ if we are a "right" module
- `useCosineCompensator`: Wheel speed correction. Set to `true`, usually.

## Notice to future maintainers!

My friends, if you change the hardware in the robot, please update `DrivetrainYAGSL` in its subsystem directory.
At the time of writing, YAGSL doesn't log everything I wanted from it (raw gyro heading, voltage/current to motors,
etc.) which is why I cast directly to the `SparkMax`/`AHRS` hardware classes. If YAGSL exposes all data in the future,
there is no need to have a difference between `DrivetrainYAGSL` and `DrivetrainSim`.

[^1]: I called this $x$ instead of $y$, because in WPILib, the $+X$ axis is forwards, and the $+Y$ axis is facing left.