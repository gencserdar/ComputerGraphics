# PA1 - Ray Tracer

This project implements a basic ray tracer with:

- Triangle mesh rendering
- Point & directional lights with shadows
- Phong shading (ambient, diffuse, specular)
- Texture mapping with bilinear sampling
- Recursive mirror reflections
- Reinhard tone mapping + gamma correction (You can change postprocessing mode in "raytracer.cpp" file)
- Multithreaded rendering (While rendering equal amount of rows are assigned to each thread)

---------------------------------------------------------------------------------

Please do not change locations/names of files or directories.

---------------------------------------------------------------------------------

## HOW TO COMPILE AND RUN:
--------------------------

- Open terminal in this directory.

- You can enter terminal commands to run the program.

### Terminal Commands:
	make: 			Compiles, runs and deletes .exe file afterwards.
	make compile: 	Compiles files. It uses threads so it may not work with old GCC versions.
	make run: 		Runs the .exe file.
	make clean: 	Deletes .exe file. This works only in Windows OS.

---------------------------------------------------------------------------------

- You can make changes on scene.xml file in this directory.

- Texture images must be located inside textureImages directory, you can change the texture image that will be sampled by changing the line:
"<textureimage>textureImages/your_texture_image_name.png</textureimage>" which is inside the scene.xml file.

- This program outputs two images: One in PNG format and one in PPM format. They both will be saved into outputImages directory after the rendering ends successfully.

---------------------------------------------------------------------------------
## External Libraries

- This program uses "tinyxml2" to parse scene.xml file into scene class.

- This program uses "stb_image" to read texture images, and it uses "stb_image_write" to write image in PNG format.

- These files are crucial and located inside src folder.