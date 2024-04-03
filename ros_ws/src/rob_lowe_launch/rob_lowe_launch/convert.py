import matplotlib 

count = 3

pgm_file = open('course.pgm', 'rb')

'''while count > 0: 
	read = pgm_file.readline()
	print(read)
	count = count - 1 
'''
def read_pgm(pgm_file): 
     assert pgm_file.readline() == b'P5\n'
     (width, height) = [int(i) for i in pgm_file.readline().split()]
     depth = int(pgm_file.readline())
     assert depth <= 255 
     
     raster = []
     for y in range(height): 
        row = []
        for y in range(width): 
            row.append(ord(pgm_file.read(1))/2.55)
        raster.append(row)
     return raster 
     
     
print(read_pgm(pgm_file))
