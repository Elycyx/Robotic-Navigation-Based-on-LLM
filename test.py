import os

print(os.stat('code.txt').st_mtime)

#1694055164.77766
with open('code.txt', 'r') as f:
    code = f.read()
f.close()
print('code:' + code)
print(os.stat('code.txt').st_mtime)