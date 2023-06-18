import random
import time

class LightFilter:

    NUM_SAMPLES = 100 # The number of previous samples + the current sample to store.

    def __init__(self):
        self.samples = []
    
    def reset(self):
        self.samples = []
    
    def set_coefs(self, coefs):
        self.coefs = coefs
    
    def add_sample(self, reading):
        if len(self.samples) == self.NUM_SAMPLES:
            self.samples.pop(0)
        self.samples.append(reading)
    
    def light(self):
        val = 0
        for i in range(len(self.samples)):
            val += self.samples[i] * self.coefs[i]
        return val

if __name__ == '__main__':
    f = open('coefs.txt', 'r')
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
    # for coef in coefs:
    #     print(coef)
    # print(len(coefs))
    filter = LightFilter()
    filter.set_coefs(coefs)
    while True:
        sample = 2.5 + random.random()
        filter.add_sample(sample)
        print(sample, filter.light())
        time.sleep(0.1)