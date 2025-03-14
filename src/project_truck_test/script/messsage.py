with open('/home/hmh/project_truck/src/project_truck/Data/output.txt', 'r') as infile, open('/home/hmh/project_truck/src/project_truck/Data/output1.txt', 'w') as outfile:
    for line in infile:
        modified_line = line.replace(' ', ',')
        outfile.write(modified_line)