from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
import math

from scipy.signal import butter, sosfilt

#ai
def highpass_filter(data, cutoff=0.5, fs=40.0, order=4):

    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    
    # Create the filter using Second-Order Sections for stability
    sos = butter(order, normal_cutoff, btype='high', analog=False, output='sos')
    filtered_data = sosfilt(sos, data)
    return filtered_data
##end ai

IMUMSG_DEF = """
std_msgs/Header header
sensor_msgs/Imu imu
sensor_msgs/MagneticField mag_field
string raw_line
"""

BASE_DIR = Path(__file__).resolve().parent 
DATA_DIR = BASE_DIR.parent / "nov10_data"        

add_types = get_types_from_msg(IMUMSG_DEF, "imu_msgs/msg/IMUmsg")
typestore = get_typestore(Stores.ROS2_JAZZY)
typestore.register(add_types)


def preprocess_accel(accel_x,accel_y,accel_z,timestamps):

    relative_timestamps = np.array([(t - timestamps[0]) / 1e9 for t in timestamps])

    accel_x = np.array(accel_x)
    accel_y = np.array(accel_y)
    accel_z = np.array(accel_z)

    mask = relative_timestamps < 1.5
    noise_x = np.mean(accel_x[mask])
    noise_y = np.mean(accel_y[mask])
    noise_z = np.mean(accel_z[mask])

    accel_x=accel_x-noise_x
    accel_y=accel_y-noise_y
    accel_z=accel_z-noise_z
    accel_v = np.array([accel_x, accel_y, accel_z]) 
    norm_accel = np.linalg.norm(accel_v, axis=0)

    sample_frequency=100

    norm_accel_centered = norm_accel - np.mean(norm_accel)
    norm_accel_filtered = highpass_filter(norm_accel, cutoff=0.5, fs=40.0, order=4)
    fft_values = np.fft.fft(norm_accel_filtered)
    fft_freqs = np.fft.fftfreq(len(norm_accel_filtered), d=1/40) #imu uses 40hz sampling
    positive_mask = fft_freqs >= 0
    frequencies = fft_freqs[positive_mask]
    magnitudes = np.abs(fft_values)[positive_mask] / len(norm_accel) * 2  


