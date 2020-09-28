import time
import numpy as np
listing = 1000 * np.random.randn(1000000)
pre_time = time.time()
std = np.std(listing)
end_time = time.time() - pre_time
print(end_time)
# 10ms with 1million entries
