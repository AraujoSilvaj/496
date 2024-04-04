import matplotlib 

count = 3

pgm_file = open('/496/ros_ws/src/rob_lowe_launch/include/course.pgm', 'rb')
yaml = open('output.yaml', 'w')

def read_pgm(pgm_file): 
     assert pgm_file.readline() == b'P5\n'
     (width, height) = [int(i) for i in pgm_file.readline().split()]
     depth = int(pgm_file.readline())
     assert depth <= 255 
     
     for y in range(height): 
        yaml.write("[ ")
        for y in range(width):
            value = ord(pgm_file.read(1))
            scaled_value = int(value)/2.55
            yaml.write(str(scaled_value) + " ")
        yaml.write(" ]")
        yaml.write("\n")

read_pgm(pgm_file)
yaml.close()
#print(read_pgm(pgm_file))
