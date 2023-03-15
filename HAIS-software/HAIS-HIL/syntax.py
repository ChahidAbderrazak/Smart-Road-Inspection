import signal
import time, sys

def timeout_handler(num, stack):
    print("\n\n - Timeout flag!!")
    raise Exception("FUBAR")

def long_function(duration):
    print("LEEEEROYYY JENKINSSSSS!!!")
    time.sleep(duration)
    print('ended')

signal.signal(signal.SIGALRM, timeout_handler)
duration=1
while True:
	signal.alarm(10)
	duration=2*duration

	try:
		# print("Before: %s" % time.strftime("%M:%S"))
		long_function(duration)
	except Exception as e:
		if "FUBAR" in str(e):
			print("\n - Frame Time out! \n Exiting the program forcedly")
			sys.exit()
		else:
			print(f' Code error: Exception: {e}')
	
	finally:
		# signal.alarm(0)
		print(f"\n - [fs={duration}]Frame downloaded  at: %s" % time.strftime("%M:%S"))