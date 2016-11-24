############################################################################
#                                                                          #
#           Musical Note Identification                                    #
#                                                                          #
############################################################################

import numpy as np
import wave
from scipy.io import wavfile
import struct
import math
from sympy.ntheory import factorint

FACTOR_LIMIT = 100

def bestFFTlength(n):
    while max(factorint(n)) >= FACTOR_LIMIT:
        n -= 1
    return n

#Function To Detect Silence
def detect_silence(sound_file):           # importing Sound file array
    i = 0                                 # i is counter for 'elements in sound file array' times iterrations
    j = 0                                 # j is number of silence detected
    silence = []                          # silence is a n_d_array whose 1st element would be silence_start and 2nd element would be silence_end
    silence_start = []                    # silence_start contains starting indices of the silence
    silence_end = []                      # silence_end contains ending indices of the silence
    cError = 0.0005                       # cError is defined as a value 'close to 0'
    while i < len(sound_file)-1:          # loop will be executed required number of times
        if sound_file[i] < cError:        # if element is close to zero it could be starting of silence
            count = i                     # saving 'possible starting index of silence' as count
            mean = sound_file[i]          # mean is 'actual mean' of consective 2 elements
            while mean < cError and i < len(sound_file)-1:          # while mean is close to zero and index is less than 'elements in sound_file' loop will be executed
                mean = (sound_file[i] + sound_file[i+1])/2          # updating mean as 'actual mean' of consective 2 elements
                i += 1                                              # updating i
            if i - count > 500:                                     # if difference of count and i ('possible ending index of silence') is greater than 500, it will be our silence
                silence_start.append(count)                         # adding count in silence_start
                silence_end.append(i)                               # adding i in silence_end
                j = j+1                                             # updating j
        i = i+1                                                     # updating i
    silence.append(silence_start)   # adding silence_start as an element in silence
    silence.append(silence_end)     # adding silence_end as an element in silence
    return silence                  # retuning silence n_d_array

#Function To detect Indices of Notes
def detect_note(sound_file):        # importing sound file array
    silence = detect_silence(sound_file)        # calling detect_silence to get silence matrix
    i = 1                                       # i is counter for 'elements in sound_file' times iterrations
    notes = []                                  # notes is a n_d_array whose 1st element would be notes_start and 2nd element would be notes_end
    notes_start = []                            # notes_start contains starting indices of the notes
    notes_end = []                              # notes_end contains ending indices of the notes
    notes_start.append(0)                       # inserting 0 as first element of notes_start
    notes_end.append(silence[0][0])             # inserting 1st element of silence_start as 1st element of notes_end
    while i < len(silence[0]):                  # loop will be executed 'number of silence found' times
        notes_start.append(silence[1][i-1])     # notes_start will be having same value as silence_end
        notes_end.append(silence[0][i])         # notes_end will be having same value as silence_start
        i = i+1                                 # updating i
    if silence[1][len(silence[0])-1] != len(sound_file)-1:  # checking if sound_file is ending with silence
        notes_start.append(silence[1][i-1])                 # if not than adding the 'ending index' of last silence to 'starting index' of next note
        notes_end.append(len(sound_file)-1)                 # adding 'ending index' of sound_file to 'ending index' of next note
    notes.append(notes_start)                   # adding notes_start as an element in notes
    notes.append(notes_end)                     # adding notes_end as an element in notes
    return notes                                # returning notes n_d_array

sampling_freq = 44100                           # Sampling frequency of audio signal

#Functon to detect notes
def play(sound_file):                           # importing 'sound_file'
    file_length = sound_file.getnframes()       # finding the number of discret vectors
    sound = np.zeros(file_length)               # defining sound array of size equal to that of sound_file with all entries equal 0
    for i in range(file_length):                # 'number of vectors in sound_file' times iterration
        data = sound_file.readframes(1)         # collecting 1 audio_frame at a time from sound_file
        data = struct.unpack("<h", data)        # converting the audio_frame from 'little endian format' to an integer corresponding to the specified byte string
        sound[i] = int(data[0])                 # integer obtained from above transformations is stored in sound file array
    sound = np.divide(sound, float(2**15))      # dividing sound file array to normalise it

    notes = detect_note(sound)                  # calling detect_note function to get the 'indicies of notes' matrix
    i = 0                                       # i is the index of 'notes' matrix
    j = len(notes[0])                           # j is the size of 'notes' matrix
    fre = []                                    # a 1_d array to contain frequency of corresponding notes
    while i < j:                                # loop till index become equal to max index of note
        sample_sound = sound[notes[0][i]:notes[1][i]]                              # defining sample_sound as a subarray of sound file array containing elements ranging from index note_start(x) to note_end(x)
        fast = abs(np.fft.fft(sample_sound,bestFFTlength(len(sample_sound))))                                       # applying Fast Fourier Transform to sample_sound and its absolute value is stored in an array named fast
        fast_max = max(fast)                                                       # defining fast_max as the maximum element in array fast
        count = 1                                                                  # defining count equal 1
        while count < len(sample_sound) and abs(fast_max - fast[count]) >= 1:      # calculating 1st index corresponding to fast_max
            count = count+1                                                        # updating count
        frequency = sampling_freq*(count)/len(sample_sound)                        # calculating frequency as product of index(max) and ratio of sampling_frequency to length of sample_sound
        fre.append(frequency)                                                      # adding frequency to 'fre' array
        i = i+1                                                                    # updating i
        Identified_Notes = []                                                      # declaring an empty list Identified_Notes
    for x in range(0,len(fre)):          
        if fre[x] > 1035 and fre[x] < 1055:
            Identified_Notes.append('C6')
        elif fre[x] > 1165 and fre[x] < 1185:
            Identified_Notes.append('D6')
        elif fre[x] > 1310 and fre[x] < 1330:   ##########################################################################
            Identified_Notes.append('E6')       #                                                                        #
        elif fre[x] > 1385 and fre[x] < 1405:   #                                                                        #
            Identified_Notes.append('F6')       #     Updating array 'Identified_Notes' with the value corresponding     #
        elif fre[x] > 1560 and fre[x] < 1580:   #                    to the value in 'fre' n_d_array                     #
            Identified_Notes.append('G6')       #                                                                        #
        elif fre[x] > 1750 and fre[x] < 1770:   #        Taking a band of (+)(-)10 hz of frequency for each note         #
            Identified_Notes.append('A6')       #                                                                        #
        elif fre[x] > 1965 and fre[x] < 1985:   #                                                                        #
            Identified_Notes.append('B6')       #                                                                        #
        elif fre[x] > 2085 and fre[x] < 2105:   ##########################################################################
            Identified_Notes.append('C7')
        elif fre[x] > 2340 and fre[x] < 2360:
            Identified_Notes.append('D7')
        elif fre[x] > 2625 and fre[x] < 2645:
            Identified_Notes.append('E7')
        elif fre[x] > 2780 and fre[x] < 2800:
            Identified_Notes.append('F7')
        elif fre[x] > 3125 and fre[x] < 3145:
            Identified_Notes.append('G7')
        elif fre[x] > 3510 and fre[x] < 3530:
            Identified_Notes.append('A7')
        elif fre[x] > 3940 and fre[x] < 3960:
            Identified_Notes.append('B7')
        elif fre[x] > 4175 and fre[x] < 4195:
            Identified_Notes.append('C8')
        elif fre[x] > 4690 and fre[x] < 4710:
            Identified_Notes.append('D8')
        elif fre[x] > 5265 and fre[x] < 5285:
            Identified_Notes.append('E8')
        elif fre[x] > 5580 and fre[x] < 5600:
            Identified_Notes.append('F8')
        elif fre[x] > 6260 and fre[x] < 6280:
            Identified_Notes.append('G8')
        elif fre[x] > 7030 and fre[x] < 7050:
            Identified_Notes.append('A8')
        elif fre[x] > 7890 and fre[x] < 7910:
            Identified_Notes.append('B8')
    return Identified_Notes                       # returning the array Identified_Notes

############################## Read Audio File #############################

if __name__ == "__main__":
    '''
    #code for checking output for single audio file
    sound_file = wave.open('Audio_files/audacity_1.wav', 'r')          # Opening Audio file
    Identified_Notes = play(sound_file)                             # Calling play function
    sound_file.close()                                              # closing the audio file
    print ("Notes = ", Identified_Notes)                            # printing the values of notes obtained for 'play'
    '''
    #code for checking output for all audio files
    Identified_Notes_list = []                                      # defining an empty list
    for file_number in range(1,6):                                  # 'number of audio_file' times iterrations
        file_name = "Audio_files/Audio_"+str(file_number)+".wav"    # defining variable file_name containing the name of corresponding audio_file
        sound_file = wave.open(file_name,'r')                       # opening audio files
        Identified_Notes = play(sound_file)                         # calling play function
        Identified_Notes_list.append(Identified_Notes)              # inserting Indetified_Notes of an audio_file into list
        sound_file.close()                                          # closing the audio file
    print (Identified_Notes_list)                                   # printing Identified_Notes_list
    
    #'''