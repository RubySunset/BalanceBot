import random
import time
import math

import numpy as np
import matplotlib.pyplot as plt
from collections import deque

class FIRFilter:

    def __init__(self):
        self.samples = []
    
    def reset(self):
        self.samples = []
    
    def set_coefs(self, coefs):
        self.coefs = coefs
        self.NUM_SAMPLES = len(coefs)
    
    def add_sample(self, reading):
        if len(self.samples) == self.NUM_SAMPLES:
            self.samples.pop(0)
        self.samples.append(reading)
    
    def filter(self):
        val = 0
        for i in range(len(self.samples)):
            val += self.samples[i] * self.coefs[self.NUM_SAMPLES - 1 - i]
        return val

class IIRFilter:

    def __init__(self):
        self.input_samples = []
        self.output_samples = []
    
    def reset(self):
        self.input_samples = []
        self.output_samples = []
    
    def set_input_coefs(self, input_coefs):
        self.input_coefs = input_coefs
        self.NUM_SAMPLES = len(input_coefs)
    
    def set_ouput_coefs(self, output_coefs):
        self.output_coefs = output_coefs
    
    def input_sample(self, reading):
        if len(self.input_samples) == self.NUM_SAMPLES:
            self.input_samples.pop(0)
        self.input_samples.append(reading)
    
    def output_sample(self, reading):
        if len(self.output_samples) == self.NUM_SAMPLES - 1:
            self.output_samples.pop(0)
        self.output_samples.append(reading)
    
    def filter(self):
        if len(self.input_samples) < self.NUM_SAMPLES:
            return 0
        else:
            val = 0
            for i in range(len(self.input_samples)):
                val += self.input_samples[i] * self.input_coefs[i]
            for i in range(len(self.output_samples)):
                val -= self.output_samples[i] * self.output_coefs[i]
            return val

def read_fir_coefs(filename):
    f = open(filename, 'r')
    text = f.read()
    f.close()
    coefs = []
    buf = ''
    for char in text:
        if char == ';':
            coefs.append(float(buf))
            buf = ''
        else:
            buf += char
    if buf != '':
        coefs.append(float(buf))
    return coefs

def read_iir_coefs(filename):
    f = open(filename, 'r')
    text = f.readlines()
    f.close()
    coefs_array = []
    for line in text:
        print(line)
        coefs = []
        buf = ''
        for char in line:
            if char == ';':
                coefs.append(float(buf))
                buf = ''
            else:
                buf += char
        if buf != '':
            coefs.append(float(buf))
        coefs_array.append(coefs)
    return coefs_array[0], coefs_array[1]

if __name__ == '__main__':

    # filter20 = FIRFilter()
    # filter20.set_coefs(read_fir_coefs('maze/coefs20.txt'))
    # filter100 = FIRFilter()
    # filter100.set_coefs(read_fir_coefs('maze/coefs100.txt'))

    # raw_samples = deque(maxlen=100)
    # filter20_samples = deque(maxlen=100)
    # filter100_samples = deque(maxlen=100)
    # i = 0
    # while True:
    #     sample = 300*math.sin(0.02*math.pi*i) + np.random.normal(0, 50)
    #     raw_samples.append(sample)
    #     filter20.add_sample(sample)
    #     filter100.add_sample(sample)
    #     filter20_samples.append(filter20.filter())
    #     filter100_samples.append(filter100.filter())
    #     plt.plot(raw_samples, 'r')
    #     plt.plot(filter20_samples, 'g')
    #     plt.plot(filter100_samples, 'b')
    #     # plt.scatter(range(len(queue)), queue)
    #     plt.xlabel('Time (1/100 second)')
    #     plt.ylabel('Reading')
    #     plt.ylim(-400, 400)
    #     plt.draw()
    #     plt.pause(0.0002)
    #     plt.clf()
    #     i += 1
    
    # filter = IIRFilter()
    # input_coefs, output_coefs = read_iir_coefs('maze/iir_coefs.txt')
    # filter.set_input_coefs(input_coefs)
    # filter.set_ouput_coefs(output_coefs)

    # input_samples = deque(maxlen=100)
    # output_samples = deque(maxlen=100)
    # i = 0
    # while True:
    #     sample = 300*math.sin(0.02*math.pi*i) + np.random.normal(0, 50)
    #     input_samples.append(sample)
    #     filter.input_sample(sample)
    #     filtered_sample = filter.filter()
    #     filter.output_sample(filtered_sample)
    #     output_samples.append(filtered_sample)
    #     plt.plot(input_samples, 'r')
    #     plt.plot(output_samples, 'g')
    #     # plt.plot(filter100_samples, 'b')
    #     # plt.scatter(range(len(queue)), queue)
    #     plt.xlabel('Time (1/100 second)')
    #     plt.ylabel('Reading')
    #     plt.ylim(-400, 400)
    #     plt.draw()
    #     plt.pause(0.0002)
    #     plt.clf()
    #     i += 1

    lf_filter = FIRFilter()
    mf_filter = FIRFilter()
    slow_filter = FIRFilter()
    lf_filter.set_coefs(read_fir_coefs('maze/coefs20.txt'))
    mf_filter.set_coefs(read_fir_coefs('maze/minphase_coefs.txt'))
    slow_filter.set_coefs(read_fir_coefs('maze/coefs100.txt'))
    print(lf_filter.NUM_SAMPLES, mf_filter.NUM_SAMPLES)
    input_samples = deque(maxlen=100)
    clean_samples = deque(maxlen=100)
    lf_samples = deque(maxlen=100)
    mf_samples = deque(maxlen=100)
    slow_samples = deque(maxlen=100)
    i = 0
    while True:
        clean = 300*math.sin(0.04*math.pi*i) # 2Hz sinusoid
        sample = clean + np.random.normal(0, 40)
        input_samples.append(sample)
        clean_samples.append(clean)
        lf_filter.add_sample(sample)
        mf_filter.add_sample(sample)
        slow_filter.add_sample(sample)
        lf_samples.append(lf_filter.filter())
        mf_samples.append(mf_filter.filter())
        slow_samples.append(slow_filter.filter())
        plt.plot(input_samples, color='red')
        plt.plot(clean_samples, color='magenta')
        plt.plot(lf_samples, color='green')
        plt.plot(slow_samples, color='blue')
        # plt.scatter(range(len(queue)), queue)
        plt.xlabel('Time (1/100 second)')
        plt.ylabel('Reading')
        plt.ylim(-400, 400)
        plt.draw()
        plt.pause(0.0002)
        plt.clf()
        i += 1