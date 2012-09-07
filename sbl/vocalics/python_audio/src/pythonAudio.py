#!/usr/bin/env python


# Importing appropriate packages
import roslib
roslib.load_manifest('pythonAudio')
import pyaudio
import wave
import sys
import struct
import numpy
import rospy
import math
from std_msgs.msg import String

# Publish to node mic

pub = rospy.Publisher('audiodata', String)
rospy.init_node('mic')

# Parameters for the input stream

chunk = 1024
FORMAT = pyaudio.paInt32
CHANNELS = 1
RATE = 44100
RECORD_SECONDS = 30
WAVE_OUTPUT_FILENAME = "output.wav"

max_int = 2147483647

# Creating stream
p = pyaudio.PyAudio()

# target_frequency = input( 'Please enter the frequency of your trigger wave: ' )

all = []

# Opening stream

stream = p.open ( format = FORMAT,
    channels = CHANNELS, 
    rate = RATE,
    input = True,
    frames_per_buffer = chunk )

rospy.loginfo("* recording")

while( not rospy.is_shutdown() ):
    all = []
    data = stream.read(chunk) # Reading in the data in a chunck from the buffer
    samps = numpy.fromstring( data, dtype = numpy.int32) # converting data into integers

    if not rospy.is_shutdown(): # Publish time
      # str = "time: %s"%rospy.get_time()
      # rospy.loginfo(str)
      for i in range(len(samps)):
        pub.publish(int(samps[i]))
        fract_level = float(max_int - abs(samps[i])) / float(max_int)
        if fract_level != 0:
          sound_level = 10*math.log( float(fract_level), 10 )
        else:
          sound_level = -40
        pub.publish(float(sound_level))
        pub.publish(float(fract_level))
        str = "Sound integer level: %i. "%samps[i] + "Sound intensity: %f."%sound_level
        rospy.loginfo(str)

    fft = numpy.fft.fft( samps ) # Taking Fourier Transform of each block of data
    afft = numpy.absolute(fft) # Taking absolute value of FFT
    frequency = []
    freq_amp = []

    for i in range(len(afft)): # Dividing frequency range into descrete levels
      frequency.append(i * (RATE/chunk))
      # freq_index = numpy.argmax(afft) # Index of the strongest frequency

    # dom_frequency = frequency[freq_index] # Storing the dominant frequency
    # flag = 0
    # flagToo = 0

    # if ( abs( dom_frequency - target_frequency ) < 50 ): # Checking to see if certain tone is detected
      # if flag == 0:
        # print "Sine wave detected"
        # flag = flag + 1

    # else:
      # if flagToo == 0:
        # print "Sine wave not detected"
        # flagToo = flagToo + 1

    all.append(data)

# print "* done recording"
stream.close()
p.terminate()
# write data to WAVE file
data = ''.join(all)
wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
wf.setnchannels(CHANNELS)
wf.setsampwidth(p.get_sample_size(FORMAT))
wf.setframerate(RATE)
wf.writeframes(data)
wf.close()
