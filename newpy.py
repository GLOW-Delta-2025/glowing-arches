import time

millis_last_sent = time.time_ns()
time.sleep(0.1)
millis_last_sent2 = time.time_ns()
print((millis_last_sent2-millis_last_sent)/1000000)
