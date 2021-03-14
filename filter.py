data = []

with open('flight_data.txt','r') as fh:
    for line in fh:
        data.append(line)

contents = []
points = []
i = 0
for line in data:
    contents.append(line.split(','))

for c in contents:
    for d in c:
        out = d.split(':')
        for o in out:
            print(o.strip(), end = ',')
    print()
