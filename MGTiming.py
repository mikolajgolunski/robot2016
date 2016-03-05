import atexit
from time import perf_counter, asctime
from datetime import timedelta


def log(s, elapsed=None):
    line = "="*40
    print(line)
    print(asctime(), '-', s)
    if elapsed:
        print("Elapsed time:", timedelta(seconds=elapsed))
    print(line)
    print()


def endlog():
    end = perf_counter()
    elapsed = round(end-start, 2)
    log("End Program", elapsed)

start = perf_counter()
atexit.register(endlog)
log("Start Program")