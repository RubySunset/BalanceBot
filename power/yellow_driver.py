#FOR YELLOW

from machine import Pin, ADC, PWM
import time
import network
import uasyncio as asyncio
import uaiohttpclient as aiohttp
import ujson
import gc
import math

ssid = 'BalanceBot'
password = 'Ajanthan'

vret_pin = ADC(Pin(26))
vout_pin = ADC(Pin(28))
vin_pin = ADC(Pin(27))
pwm = PWM(Pin(0))
pwm.freq(100000)
pwm_en = Pin(1, Pin.OUT)

pwm_out = 0
pwm_ref = 0
setpoint = 0.0
delta = 0.05

VOLTAGE_CONST = 5 / 65535 * 3.3

pwm_ref = 25000
pwm_step = 1

SET_CURRENT = 0.2
pwm_enable = 0

def connect():
    #Connect to WLAN
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while wlan.isconnected() == False:
        print('Waiting for connection...')
        time.sleep(1)
    print(wlan.ifconfig())
    
def saturate(duty, maxduty, minduty):
    if duty > maxduty:
        duty = maxduty
    if duty < minduty:
        duty = minduty
    return duty
    
    
async def poll_lights():
    global pwm_enable
    last_request_time = time.time()
    while True:
        if time.time() - last_request_time >= 5:
            last_request_time = time.time()
            response = await aiohttp.request("GET", "http://api.balancebot.site/beacon")
            
            if response.status == 200:
                should_switch_on = ujson.loads(await response.read())
                print(should_switch_on)
                if (should_switch_on):
                    pwm_enable = 1
#                     print("pwm enable on")
                else:
                    pwm_enable = 0
#                 print("Vret: ", vret)
#             else:
#                 print("Failed to get response.")
            gc.collect()
        else:
            await asyncio.sleep(1)
            
            
async def control_pwm():
    global pwm_enable
    VOLTAGE_CONST = 5 / 65535 * 3.3
    DUTY_CONST = 2*15
    SET_VOLTAGE_MAX = 2.2
    SET_VOLTAGE_MIN = 1.8
    pwm_ref = 25000
    pwm_step = 10
    count = 0
    
#     pwm_enable = 0

    while True:
        

        vin = vin_pin.read_u16() * VOLTAGE_CONST
        vout = vout_pin.read_u16() * VOLTAGE_CONST
        vret = vret_pin.read_u16() * VOLTAGE_CONST
        
        if vret > SET_CURRENT:
            pwm_ref -= pwm_step
        if vret < SET_CURRENT:
            pwm_ref += pwm_step

        if not pwm_enable:
            pwm_en.value(0)
            pwm_ref = 0
            minduty = 100
        else:
            pwm_en.value(1)
            minduty = 10000
            
        if vin < 7:
            vin = 7
#         print(DUTY_CONST*vret/(vin*(vin/SET_VOLTAGE-1)))
        
        maxduty = int(math.sqrt(DUTY_CONST*vret/(vin*(vin/SET_VOLTAGE_MAX-1)))*65535)
#         minduty = int(math.sqrt(DUTY_CONST*vret/(vin*(vin/SET_VOLTAGE_MIN-1)))*65535*0.5)
#         minduty = 10000
#         maxduty = 65535
        pwm_out = saturate(pwm_ref, maxduty, minduty)
        pwm.duty_u16(pwm_out)
        
    
        if count > 2000:
            print(vin, vout, vret, pwm_out, maxduty, minduty)
            count = 0
        count += 1
#         print(vin, vout, vret, pwm_out, maxduty)

        await asyncio.sleep(0)  # Yield control to the event loop
        

if __name__ == '__main__':
    try:
        connect()
        asyncio.run(asyncio.gather(control_pwm(), poll_lights()))
    except KeyboardInterrupt:
        print("stopped")
