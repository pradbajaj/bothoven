############################################################################
#																		   #
#			Musical Note Identification							   #
#																		   #
############################################################################

# The code as given in task1 folder
# Incorporate your code here

import numpy as np
import wave
import struct

#Teams can add other helper functions
#which can be added here


sampling_freq = 44100	# Sampling frequency of audio signal

def play(sound_file):
    '''
    sound_file-- a single test audio_file as input argument
    
    #add your code here

    '''
        
    return Identified_Notes

############################## Read Audio File #############################

if __name__ == "__main__":
    #code for checking output for single audio file
    sound_file = wave.open('Test_Audio_files/Audio_1.wav', 'r')
    Identified_Notes = play(sound_file)
    print "Notes = ", Identified_Notes

    #code for checking output for all images
    Identified_Notes_list = []
    for file_number in range(1,6):
        file_name = "Test_Audio_files/Audio_"+str(file_number)+".wav"
        sound_file = wave.open(file_name)
        Identified_Notes = play(sound_file)
        Identified_Notes_list.append(Identified_Notes)
    print Identified_Notes
    


