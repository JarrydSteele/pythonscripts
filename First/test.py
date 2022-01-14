
x = b"\xc8"
print(x.decode('utf-8'))

# x = eval("b'\\xfe\\xff\\x000\\x008\\x00/\\x001\\x002\\x00/\\x001\\x009\\x009\\x003'")

# print(x)   ### b'\xfe\xff\x000\x008\x00/\x001\x002\x00/\x001\x009\x009\x003'

# print(x.decode('utf-16'))  ### returns 08/12/1993