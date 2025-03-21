import serial

# Define ranges of the camera's coordinates (example, needs calibration)
CAMERA_X_MIN = 0
CAMERA_X_MAX = 1280
CAMERA_Y_MIN = 0
CAMERA_Y_MAX = 720

# Supported colors with their corresponding RGB codes
COLOR_RGB = {
    "RED": (255, 0, 0),
    "BLUE": (0, 0, 255),
    "GREEN": (0, 255, 0),
    "WHITE": (255, 255, 255),
}

# --- DMX Calibration ---
DMX_CALIBRATION = {
    1: { # DMX Light 1
        "LEFT": 70,
        "RIGHT": 105,
        "TOP": 0,
        "BOTTOM": 105
    },
    2: { # DMX Light 2
        "LEFT": 70,
        "RIGHT": 105,
        "TOP": 0,
        "BOTTOM": 105
    }
}

def camera_to_dmx(x, y, light_number):
    """
    Converts camera X, Y coordinates to DMX pan and tilt values for a specific light.

    x, y: Camera coordinates
    light_number: The number of the light for which the calculation is being done
    Returns: (pan, tilt) values for DMX
    """
    # Get calibration values for the specified light
    if light_number not in DMX_CALIBRATION:
        raise ValueError(f"Light number {light_number} is not defined in DMX_CALIBRATION.")
    calibration = DMX_CALIBRATION[light_number]

    # Extract calibration values
    dmx_left = calibration["LEFT"]
    dmx_right = calibration["RIGHT"]
    dmx_top = calibration["TOP"]
    dmx_bottom = calibration["BOTTOM"]

    # Map screen coordinates (x, y) to DMX pan and tilt values
    pan_raw = int(dmx_right + (dmx_left - dmx_right) * (x / CAMERA_X_MAX))
    tilt_raw = int(dmx_bottom + (dmx_top - dmx_bottom) * (y / CAMERA_Y_MAX))

    pan = max(0, min(255, int(pan_raw)))
    tilt = max(0, min(255, int(tilt_raw)))

    return pan, tilt

def send_dmx(people, serial_connection):
    """
    Sends DMX pan, tilt, and color values to the Arduino serial in the format:
        '0:pan,tilt,color;1:pan,tilt,color'
    people: List of dictionaries containing `x`, `y` and `color` for each person.
            Example: [{'x': 640, 'y': 360, 'color': 'RED'}, {'x': 300, 'y': 200, 'color': 'BLUE'}]
            Max 2 items.
    serial_connection: An open Serial object to send data over.
    """
    if len(people) > 2:
        raise ValueError("Only up to 2 lights are supported.")

    num_lights = len(DMX_CALIBRATION)
    # Create a default people list with (0, 0, "WHITE") for missing lights
    default_people = [{"x": 0, "y": 0, "color": "WHITE"}] * num_lights
    for i, person in enumerate(people):
        default_people[i] = person  # Overwrite with given people data

    # Convert defaulted people to DMX pan, tilt, color format
    dmx_data = []
    for i in range(num_lights):
        light_number = i + 1  # Light index starts at 1
        person = default_people[i]

        x = person["x"]
        y = person["y"]
        color = person["color"]

        if color not in COLOR_RGB:  # Validate color
            raise ValueError(f"Invalid color '{color}'. Must be one of {list(COLOR_RGB.keys())}.")

        if i == 0:
            pan, tilt = camera_to_dmx(x, y, light_number)  # Convert camera to DMX coordinates for the first person
            r, g, b = COLOR_RGB[color]  # Get RGB values for the color
        else:
            pan = 100  # Hardcoded pan value for the rest
            tilt = 50  # Hardcoded tilt value for the rest
            r = 255,
            b = 255,
            g = 255,

        dmx_data.append(f"{i}:{pan},{tilt},{r},{g},{b}")  # Append in 'index:pan,tilt,R,G,B' format

    # Prepare string to send with a semicolon at the end
    dmx_string = ";".join(dmx_data) + ";\n"

    # Send DMX data over an already open serial connection
    try:
        serial_connection.write(dmx_string.encode())  # Send encoded string
        print(f"Sent: {dmx_string}")  # Print confirmation
    except serial.SerialException as e:
        print(f"Error sending DMX data: {e}")