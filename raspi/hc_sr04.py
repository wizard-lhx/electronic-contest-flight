import lgpio as lg
import time

h = lg.gpiochip_open(0)

lg.gpio_claim_output(h, 20, lg.LOW, lg.SET_BIAS_PULL_DOWN)
lg.gpio_claim_input(h, 25, lg.SET_BIAS_PULL_DOWN)

def distance():
    lg.gpio_write(h, 20, lg.HIGH)

    start_time = time.time()
    while time.time() - start_time < 0.00002:
        pass
    
    lg.gpio_write(h, 20, lg.LOW)

    while lg.gpio_read(h, 25) == 0:
        start_time = time.time()

    while lg.gpio_read(h, 25) == 1:
        stop_time =  time.time()

    time_elapsed = stop_time - start_time
    dis = time_elapsed * 34300 / 2

    if time_elapsed > 0.0001 and time_elapsed < 0.06:
        dis = time_elapsed * 34300 / 2
        return dis
    else:
        return None

try:
    while True:
        dis = distance()
        if dis:
            print(dis)

except KeyboardInterrupt:
    print("end")