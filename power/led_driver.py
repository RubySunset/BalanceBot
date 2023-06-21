def poll_lights(host, url):
    last_request_time = time.time()
    const = 5 / 65535 * 3.3
    pwm_ref = 25000
    pwm_step = 1
    count = 0
    pwm_enable = 0
    
    while True:
        pwm_en.value(1)

        vin = vin_pin.read_u16() * const
        vout = vout_pin.read_u16() * const
        vret = vret_pin.read_u16() * const

        if vret > SET_CURRENT:
            pwm_ref -= pwm_step
        if vret < SET_CURRENT:
            pwm_ref += pwm_step
        
        if not pwm_enable:
            pwm_en.value(0)
            pwm_ref = 0
        
        pwm_out = saturate(pwm_ref)
        pwm.duty_u16(pwm_out)
        
        if count > 2000:
            print(vin, vout, vret, pwm_out)
            count = 0
        count += 1

        if time.time() - last_request_time >= 5:
            
            last_request_time = time.time()
            response = urequests.get("{}/{}".format(host, url))
        
            if response.status_code == 200:
                should_switch_on = response.json()
                print(should_switch_on)
                if (should_switch_on):
                    pwm_enable = 1
                    print("pwm enable on")
                else:
                    pwm_enable = 0
                print("Vret: ", vret)
                
            else:
                print("Failed to get response.")
    

if __name__ == '__main__':
    try:
        connect()
        poll_lights(host, url)
    except KeyboardInterrupt:
        print("stopped")