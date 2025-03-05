# Meet Caspian

This is the robot code for our 2025 robot Caspian.

## Setup

1. If using the radios
    1. Take out the radio labeled "Laptop Access Point" or "Laptop AP". Plug it into the
       wall using the PoE wall adapter, using the "RIO" port on the radio.
    2. Connect the radio to the laptop using the "DS" port
2. Set up the RIO (this only needs to be done once after updating RIO firmware)
    1. After installing all the required software, connect to the RIO using Wi-Fi or ethernet.
    2. Find the RIO's IP address by using the FRC Driver Station, going to the 2nd tab from the top, and
       looking at the IP address next to "Robot." If nothing shows up (i.e. not green), wait until the RIO
       boots. The IP is usually `10.27.91.2` but it may be different.
    3. Open a command prompt and type `ssh admin@IP_ADDRESS`. There should not be a password.
    4. Type `cd /var/log` to navigate to the log directory, then make a folder using `mkdir akit`.
    5. Give the code permission to write to the log directory with `chown -R lvuser:everyone /var/log/akit`.
    6. Exit the command prompt
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