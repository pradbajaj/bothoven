############################################################################
#																		   #
#			Musical Note Identification							   #
#																		   #
############################################################################

import numpy as np
import wave
from scipy.io import wavfile
import struct
import math

#Teams can add other helper functions
#which can be added here
def detect_silence(sound_file):
    i = 0
    j = 0
    silence = []
    silence_start = []
    silence_end = []
    #print(len(sound_file))
    while i < len(sound_file)-1:
        if sound_file[i] == 0:
            count = i
            while sound_file[i] == 0 and i < len(sound_file)-1:
                i = i+1
            if i - count > 500:
                silence_start.append(count)
                silence_end.append(i)
                j = j+1
        i = i+1
    silence.append(silence_start)
    silence.append(silence_end)
    #print(silence)

    return silence
def detect_note(sound_file):
    silence = detect_silence(sound_file)
    i = 1
    notes = []
    notes_start = []
    notes_end = []
    notes_start.append(0)
    notes_end.append(silence[0][0])
    while i < len(silence[0]):
        notes_start.append(silence[1][i-1])
        notes_end.append(silence[0][i])
        i = i+1
    notes.append(notes_start)
    notes.append(notes_end)
    #print notes
    return notes
sampling_freq = 44100	# Sampling frequency of audio signal

def play(sound_file):
    '''
    sound_file-- a single test audio_file as input argument
    
    #add your code here

    '''
    notes = detect_note(sound_file)
    i = 0
    j = len(notes[0])
    fre = []
    while i < j:
        sample_sound = sound_file[notes[0][i]:notes[1][i]]
        #print(sample_sound)
        fast = abs(np.fft.fft(sample_sound))
        #print(fast)
        fast_max = max(fast)
        count = 1
        while count < len(sample_sound) and abs(fast_max - fast[count]) >= 1:
            count = count+1
        #print(count)
        frequency = sampling_freq*(count)/len(sample_sound)
        fre.append(frequency)
        i = i+1
    #print(fre)    
    Identified_Notes = fre
    return Identified_Notes

############################## Read Audio File #############################

if __name__ == "__main__":
    '''
    #code for checking output for single audio file
    sound_file = wave.open('Audio_files/Audio_1.wav', 'r')
    file_length = sound_file.getnframes()
    sound = np.zeros(file_length)   
    for i in range(file_length):
        data = sound_file.readframes(1)
        data = struct.unpack("<h", data)
        sound[i] = int(data[0])
    sound = np.divide(sound, float(2**15))
    #print(sound)
    Identified_Notes = play(sound)
    print ("Notes = ", Identified_Notes)
    '''
    #code for checking output for all audio files
    Identified_Notes_list = []
    for file_number in range(1,6):
        file_name = "Audio_files/Audio_"+str(file_number)+".wav"
        sound_file = wave.open(file_name,'r')
        file_length = sound_file.getnframes()
        sound = np.zeros(file_length)   
        for i in range(file_length):
            data = sound_file.readframes(1)
            data = struct.unpack("<h", data)
            sound[i] = int(data[0])
        sound = np.divide(sound, float(2**15))
        Identified_Notes = play(sound)
        Identified_Notes_list.append(Identified_Notes)
    print (Identified_Notes_list)
    
    #'''

