import wave
import struct
import numpy as np
from scipy import fft

# opens sound in read only mode
with wave.open('sound.wav', 'r') as sound_file:
	# Gets the number of sampling (loosely speaking)
	file_length = sound_file.getnframes ()			
	# Creates a new vector and intializes it to 0
	sound = np.zeros (file_length)					
	Fs = sound_file.getframerate()
	# print (file_length)
	# print (Fs)

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
	# Calculate the value of cMin 
	cMin = 50
	silenceEnd = [0]
	silenceStart = []
	Count = []
	# print ("Test1")
	# Modify Kadane's algo to detect silence
	for i in range (len(sound)):
		if i == 0:
			if count == 0:
				# Append start position of silence in a list
				silenceStart.append (i)
			count += 1
		else:
			# Appends end position of silence and the size of silence
			# Also resets count to 0
			silenceEnd.append (i)
			Count.append (count)
			count = 0
	# Detecting first note. Maintain all silence start and end point to detect multiple node
	# print ("test2")
	note_start = 0
	note_end = silenceStart
	# Not sure if this works
	fre = []
	# Find a better way to apply fft. This method is too slow
	print ("fft start")
	Sound = fft(sound)
	print ("fft end")
	# Perform the following task as long as there is more silence
	while (silenceStart):
		i = silenceEnd.pop()
		j = silenceStart.pop()
		count = Count.pop()
		if count > cMin:
			freq = max (Sound[i], Sound[j])
			fre.append(Fs*(freq-1)/file_length)
	for i in fre:
		print(i)