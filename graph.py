import serial
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.lines import Line2D

# Initialize serial port
ser = serial.Serial(
    port='COM6', baudrate=115200, 
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS )

ser.isOpen()

# Initialize plot
fig, ax = plt.subplots()
car = ax.plot([], [], marker='s', color='blue')[0]
metal_points = ax.scatter([], [], color='red', label='Metal Detection')
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.legend()

# Initialize car position
car_x = 5
car_y = 5
car.set_data([car_x], [car_y])

# Metal detection points
metal_x = []
metal_y = []

# Read and plot data
while True:
    try:
        data = ser.readline().decode().strip()
        if data.startswith('Joystick:'):
            _, direction = data.split(':')
            if direction == 'UP':
                car_y += 1
            elif direction == 'DOWN':
                car_y -= 1
            elif direction == 'LEFT':
                car_x -= 1
            elif direction == 'RIGHT':
                car_x += 1
            car.set_data([car_x], [car_y])
            plt.pause(0.01)
        elif data == 'Metal Detected':
            metal_x.append(car_x)
            metal_y.append(car_y)
            metal_points.set_offsets(zip(metal_x, metal_y))
            plt.pause(0.01)
    except KeyboardInterrupt:
        break

ser.close()
plt.show()
