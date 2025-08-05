# Connections
# MPU6050 - Raspberry Pi
# VCC - 5V  (Pin 2 or 4)
# GND - GND (Pin 6)
# SCL - SCL (Pin 5)
# SDA - SDA (Pin 3)

from Kalman import KalmanAngle
import smbus
import time
import math

# Kalman filters
kalmanX = KalmanAngle()
kalmanY = KalmanAngle()
kalmanZ = KalmanAngle()

# Constants
RestrictPitch = True  # Set to False to restrict roll instead
radToDeg = 57.2957786
kalAngleX = 0
kalAngleY = 0
kalAngleZ = 0

# MPU6050 Registers
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# Initialize MPU6050
def MPU_Init():
	bus.write_byte_data(DeviceAddress, SMPLRT_DIV, 7)
	bus.write_byte_data(DeviceAddress, PWR_MGMT_1, 1)
	bus.write_byte_data(DeviceAddress, CONFIG, int('0000110', 2))
	bus.write_byte_data(DeviceAddress, GYRO_CONFIG, 24)
	bus.write_byte_data(DeviceAddress, INT_ENABLE, 1)

def read_raw_data(addr):
	high = bus.read_byte_data(DeviceAddress, addr)
	low = bus.read_byte_data(DeviceAddress, addr + 1)
	value = ((high << 8) | low)
	if value > 32767:
		value -= 65536
	return value

# I2C bus
bus = smbus.SMBus(1)
DeviceAddress = 0x68
MPU_Init()

time.sleep(1)

# Initial read
accX = read_raw_data(ACCEL_XOUT_H)
accY = read_raw_data(ACCEL_YOUT_H)
accZ = read_raw_data(ACCEL_ZOUT_H)

if RestrictPitch:
	roll = math.atan2(accY, accZ) * radToDeg
	pitch = math.atan(-accX / math.sqrt((accY ** 2) + (accZ ** 2))) * radToDeg
else:
	roll = math.atan(accY / math.sqrt((accX ** 2) + (accZ ** 2))) * radToDeg
	pitch = math.atan2(-accX, accZ) * radToDeg

kalmanX.setAngle(roll)
kalmanY.setAngle(pitch)
kalmanZ.setAngle(0)  # Initialize yaw at 0

gyroXAngle = roll
gyroYAngle = pitch
gyroZAngle = 0

compAngleX = roll
compAngleY = pitch
compAngleZ = 0

timer = time.time()
flag = 0

while True:
	if flag > 100:
		print("There is a problem with the connection")
		flag = 0
		continue

	try:
		# Accelerometer
		accX = read_raw_data(ACCEL_XOUT_H)
		accY = read_raw_data(ACCEL_YOUT_H)
		accZ = read_raw_data(ACCEL_ZOUT_H)

		# Gyroscope
		gyroX = read_raw_data(GYRO_XOUT_H)
		gyroY = read_raw_data(GYRO_YOUT_H)
		gyroZ = read_raw_data(GYRO_ZOUT_H)

		dt = time.time() - timer
		timer = time.time()

		# Roll and Pitch
		if RestrictPitch:
			roll = math.atan2(accY, accZ) * radToDeg
			pitch = math.atan(-accX / math.sqrt((accY ** 2) + (accZ ** 2))) * radToDeg
		else:
			roll = math.atan(accY / math.sqrt((accX ** 2) + (accZ ** 2))) * radToDeg
			pitch = math.atan2(-accX, accZ) * radToDeg

		# Gyro rates
		gyroXRate = gyroX / 131
		gyroYRate = gyroY / 131
		gyroZRate = gyroZ / 131

		# Kalman Filter Updates
		if RestrictPitch:
			if ((roll < -90 and kalAngleX > 90) or (roll > 90 and kalAngleX < -90)):
				kalmanX.setAngle(roll)
				compAngleX = roll
				kalAngleX = roll
				gyroXAngle = roll
			else:
				kalAngleX = kalmanX.getAngle(roll, gyroXRate, dt)

			if abs(kalAngleX) > 90:
				gyroYRate = -gyroYRate
			kalAngleY = kalmanY.getAngle(pitch, gyroYRate, dt)
		else:
			if ((pitch < -90 and kalAngleY > 90) or (pitch > 90 and kalAngleY < -90)):
				kalmanY.setAngle(pitch)
				compAngleY = pitch
				kalAngleY = pitch
				gyroYAngle = pitch
			else:
				kalAngleY = kalmanY.getAngle(pitch, gyroYRate, dt)

			if abs(kalAngleY) > 90:
				gyroXRate = -gyroXRate
			kalAngleX = kalmanX.getAngle(roll, gyroXRate, dt)

		# Z Axis (Yaw)
		yaw = gyroZAngle + gyroZRate * dt
		kalAngleZ = kalmanZ.getAngle(yaw, gyroZRate, dt)
		gyroZAngle += gyroZRate * dt
		compAngleZ = 0.93 * (compAngleZ + gyroZRate * dt) + 0.07 * yaw

		# Normalize angles
		if abs(gyroXAngle) > 180:
			gyroXAngle = kalAngleX
		if abs(gyroYAngle) > 180:
			gyroYAngle = kalAngleY
		if abs(gyroZAngle) > 180:
			gyroZAngle = kalAngleZ

		# Print
		print(f"Angle X: {kalAngleX:.2f}  Angle Y: {kalAngleY:.2f}  Angle Z (Yaw): {kalAngleZ:.2f}")
		time.sleep(0.005)

	except Exception as exc:
		flag += 1
