import serial
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Config - change if needed
PORT = '/dev/rfcomm0'  # Your BT serial port
BAUD = 9600
TIMEOUT = 1  # seconds

ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)

# Data lists
times = []
angles = []
pwms = []
errors = []

fig, ax = plt.subplots()
line_angle, = ax.plot([], [], label='Angle (deg)', color='b')
line_pwm, = ax.plot([], [], label='PWM', color='g')
line_error, = ax.plot([], [], label='Error', color='r')

def init():
    ax.set_xlim(0, 100)  # Initial x limit
    ax.set_ylim(-100, 100)  # Adjust based on expected values
    ax.set_xlabel('Time')
    ax.set_ylabel('Value')
    ax.set_title('Real-Time Robot Data')
    ax.legend()
    return line_angle, line_pwm, line_error

def update(frame):
    line = ser.readline().decode('utf-8').strip()
    if line:
        try:
            data = json.loads(line)
            times.append(len(times))
            angles.append(data.get('angle', 0))
            pwms.append(data.get('pwm', 0))
            errors.append(data.get('err', 0))

            # Dynamic x limit
            if len(times) > 100:
                ax.set_xlim(len(times) - 100, len(times))
            else:
                ax.set_xlim(0, 100)

            # Update lines
            line_angle.set_data(times, angles)
            line_pwm.set_data(times, pwms)
            line_error.set_data(times, errors)

            # Auto-scale y
            ax.relim()
            ax.autoscale_view()
        except (json.JSONDecodeError, ValueError):
            pass
    return line_angle, line_pwm, line_error

ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=100)
plt.show()

# Close serial on exit
ser.close()