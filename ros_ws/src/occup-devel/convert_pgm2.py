from PIL import Image 

def image_to_pgm(img_path, out_path):
    img = Image.open(img_path)
    img_gray = img.convert('L')
    width, height = img_gray.size
    
    with open(out_path, 'w') as pgm_file: 
        pgm_file.write("P2\n{} {}\n255\n".format(width, height))
        
        pixel_values = img_gray.load()
        for y in range(height): 
            for x in range(width): 
                pgm_file.write("{} ".format(pixel_values[x, y]/255))
                
    print("PGM file created successfully!")
     
image_to_pgm("course.png", "outtest.pgm")
