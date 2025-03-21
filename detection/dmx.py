import serial

# Define ranges of the camera's coordinates (example, needs calibration)
CAMERA_X_MIN = 0
CAMERA_X_MAX = 1280
CAMERA_Y_MIN = 0
CAMERA_Y_MAX = 720

COLORS = ["RED", "BLUE", "GREEN"]  # Supported colors

# --- DMX 1 Calibration ---
DMX_CALIBRATION = {
    1: { # DMX Light 1
        "LEFT": 70,
        "RIGHT": 105,
        "TOP": 0,
        "BOTTOM": 105
    },
    2: { # DMX Light 2
        "LEFT": 60,
        "RIGHT": 95,
        "TOP": 0,
        "BOTTOM": 105
    }
}

def map_value(value, input_min, input_max, output_min, output_max):
    """Maps a value from one range to another."""
    return output_min + (value - input_min) * (output_max - output_min) / (input_max - input_min)


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

    # Map camera X to DMX pan range
    pan = map_value(x, CAMERA_X_MIN, CAMERA_X_MAX, calibration["LEFT"], calibration["RIGHT"])
    # Map camera Y to DMX tilt range
    tilt = map_value(y, CAMERA_Y_MIN, CAMERA_Y_MAX, calibration["TOP"], calibration["BOTTOM"])

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
    # Create a default people list with (0, 0, "RED") for missing lights
    default_people = [{"x": 0, "y": 0, "color": "RED"}] * num_lights
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

        if color not in COLORS:  # Validate color
            raise ValueError(f"Invalid color '{color}'. Must be one of {COLORS}.")

        pan, tilt = camera_to_dmx(x, y, light_number)  # Convert camera to DMX coordinates
        dmx_data.append(f"{i}:{int(pan)},{int(tilt)},{color}")  # Append in 'index:pan,tilt,color' format

    # Prepare string to send
    dmx_string = ";".join(dmx_data)

    # Send DMX data over an already open serial connection
    try:
        serial_connection.write(dmx_string.encode('utf-8'))  # Send encoded string
        print(f"Sent: {dmx_string}")  # Print confirmation
    except serial.SerialException as e:
        print(f"Error sending DMX data: {e}")