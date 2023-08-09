import lgpio as lg

def check_key_num(chip, gpio, level, timestamp):
    if gpio == 6:
        print(3)
    elif gpio == 13:
        print(2)

h = lg.gpiochip_open(0)

lg.gpio_claim_output(h, 20, lg.LOW, lg.SET_BIAS_PULL_DOWN)
lg.gpio_claim_alert(h, 6, lg.FALLING_EDGE, lg.SET_BIAS_PULL_UP)
lg.gpio_claim_alert(h, 13, lg.FALLING_EDGE,lg.SET_BIAS_PULL_UP)

lg.gpio_set_debounce_micros(h, 6, 20000)
lg.gpio_set_debounce_micros(h, 13, 20000)

cb1 = lg.callback(h, 6, lg.FALLING_EDGE, check_key_num)
cb2 = lg.callback(h, 13, lg.FALLING_EDGE, check_key_num)

try:
    while True:
        pass
except KeyboardInterrupt:
    print("end")