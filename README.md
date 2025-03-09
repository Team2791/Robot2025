# Meet Caspian

This is the robot code for our 2025 robot Caspian.

## Setup

1. If using the radios
    1. Take out the radio labeled "Laptop Access Point" or "Laptop AP". Plug it into the
       wall using the PoE wall adapter, using the "RIO" port on the radio.
    2. Connect the radio to the laptop using the "DS" port
    3. Turn on the robot and a blue light should appear on the Laptop AP.
3. Pull the code
    ```bash
    git clone https://github.com/Team2791/Robot2025
    ```
4. Open the code in VS Code
5. Deploy the code to the robot

## Other notes

- This code uses a [fork](https://github.com/onlycs/elastic-dashboard) of elastic dashboard for NT4 Struct Parsing. I
  have a link to the exe for this posted in the `#programming` channel on Slack. The fork should be merged in the 2026
  season, according to the lead developer of the project.