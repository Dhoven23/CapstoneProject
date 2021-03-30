import sys

filename = sys.argv[1]

if len(sys.argv) != 2:
    print("check usage\npython3 header.py <input.txt>")
    exit()
content = []
with open(filename,'r') as f:
    while(1):
        if(f.readable()):
            line = f.readline()
            if(line):
                content.append(line)
            else:
                break