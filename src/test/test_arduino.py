import smbus
import time
import struct
import time
 
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(command, x_value, tetha_value):
#	bus.write_byte(address, command)
#	bus.write_byte(address, value)
	bus.write_i2c_block_data(address, command, [x_value, tetha_value] )
	return -1

#def readNumber():
# number = bus.read_byte(address)
# number = bus.read_byte_data(address, 1)
# return number

def get_data():
	return bus.read_i2c_block_data(address, 0)

def get_float(data, index):   
	bytes = data[4*index:(index+1)*4]
	return struct.unpack('f', "".join(map(chr, bytes)))[0]

while True:
	command = input('Enter command:  ')
	x_value = input('Enter x value:  ')
	tetha_value = input('Enter tetha value:  ')
	if not command:
		continue

	bit = x_value.Bits(uint=byte, length=8)
	writeNumber(command, bit.unpack('int'), tetha_value)
	print 'sending:  '
	time.sleep(0.1)
	data = get_data()
	x_recieved_number = get_float(data, 0)
	theta_recieved_number = get_float(data, 1)
	print 'Receiving X: ', x_recieved_number
	print 'Receiving Theta: ', theta_recieved_number
	time.sleep(0.1)
'''
	error = 0
	i = 0
	start = time.time()
	for x in range(0, 1000):
		try:
#print "We're on time %d" % (x)
			writeNumber(var)
#print 'sending:  ', var
			time.sleep(0.002)
			data = get_data()
			old_number = number
			number = get_float(data, 0)
			if number != old_number:
				x = x - 1
				i += 1
		except:
			x = x - 1
			error += 1
			pass 
	end = time.time()
#print 'Inequality appears!!! on iteration', x
#print 'Receiving: ', number

	print 'Number of not equal values', i
	print 'Number of errors', error
	print 'Time is - ', end-start
'''

#writeNumber(var)

#	print 'sending:  ', var
# sleep one second


# number = readNumber

#data = get_data()
#	number = get_float(data, 0)
#	print 'Receiving: ', number
