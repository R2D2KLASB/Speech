import sys

a_file = open(sys.argv[1], "r")
output = "{"
for line in a_file:
    if line.strip()[0:3] == "G00":
        x = line.strip().find('X')
        y = line.strip().find('Y')
        output += "{"
        for

input("saus")
