
import csv
import os
from obelisk_py.zoo.control.example.d1.constants import *

def initialize_folder() -> bool:
    """Initializes a folder for storing joint data."""
    if not os.path.exists(FOLDER_PATH):
        os.makedirs(FOLDER_PATH)
        return True
    return False

def record_data(filepath: str, t: float, servo_data: list) -> None:
    """
    Record the time and servo data (either the command or the state)
    in the csv file at the given filepath.
    """  # noqa: D205
    row = [t] + servo_data

    # Append to CSV file
    file_exists = os.path.isfile(filepath)
    with open(filepath, 'a', newline='') as file:
        writer = csv.writer(file)
        # Write header if file is new
        if not file_exists:
            writer.writerow(HEADER)
        # Write row
        writer.writerow(row)