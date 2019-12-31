import gzip

with open('/usr/share/kbd/consolefonts/default8x16.psfu.gz', 'rb') as f:
    d = f.read()
    d = gzip.decompress(d)

d = d[0x20:0x820]

with open('font.hex', 'w') as f:
    for j, x in enumerate(d):
        num = 0
        for i in range(8):
            bit = x >> 7 - i & 1
            num |= bit << i
        f.write(f'{num:02x}\n')

fb = [0x20] * 2048

for i, x in enumerate("Hello, world!"):
    fb[i] = ord(x)

with open('fb.hex', 'w') as f:
    for x in fb:
        f.write(f'{x:03x}\n')
