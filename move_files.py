import os

for root, dirs, files in os.walk('.'):
    for fn in files:
        if 'untitled' in fn:
            os.rename(fn, f"./sl_outputs/{fn.replace('untitled', 'ss')}")



