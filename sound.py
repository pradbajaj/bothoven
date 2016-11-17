import wave
import struct
import numpy as np

# opens sound in read only mode
with wave.open('sound.wav', 'r') as sound_file:
	# Gets the number of sampling (loosely speaking)
	file_length = sound_file.getnframes ()			
	# Creates a new vector and intializes it to 0
	sound = np.zeros (file_length)					

	for i in range (file_length):
		# Reads each sample and stores it in an array
		data = sound_file.readframes (1)			
		# Takes the format of the byte and treats it as integer in little endian format
		data = struct.unpack ("<h", data)			
		# Stores the newly computed integer value in an array
		sound[i] = int (data[0])					
	# Divides the integer to normalize data in the range [-1.0, 1.0]
	sound = np.divide (sound, float (2**15))		

	count = 0
	# Calculate this
	cMax = 2222
	sRange = 0.05
	# Modify Kadane's algo to detect silence
	for i in range (len(sound)):
		if (sound[i] > -sRange && sound[i] < sRange):
			count++;
		else:
			count = 0;
		if (count > cMax):
