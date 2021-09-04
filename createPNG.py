#http://blender.stackexchange.com/questions/24133/modify-obj-after-import-using-python

import bpy
import sys

inFolder = "./build/out"  
resultFolder = "./build/png"  
nameFileIn = "out_object{:02d}_"
nameFileOut = "img_"

argv = sys.argv
argv = argv[argv.index("--") + 1:]
	

def delete(objects):
	for o in objects:
		#SelectOnlyGivenObject(o)
		bpy.ops.object.select_all(action='DESELECT')
		o.select_set(True)
		bpy.ops.object.delete()

def makesPNGs(firstFrame, lastFrame, firstObject, lastObject):
	colors = []
	for objInd in range(firstObject, lastObject):
		colors.append(bpy.data.materials.new(str(objInd)))
		print("colors: ", colors)
		colors[objInd].diffuse_color = (0.1, 0.0, 0.7, 0.3)

	for frame in range(firstFrame, lastFrame):
		pathMesh = inFolder + "/" + nameFileIn + str(frame).zfill(1) + ".obj"
		print("this is the name I get: ", pathMesh)
		prior_objects = [object for object in bpy.context.scene.objects]
		for objInd in range(firstObject, lastObject):
			print("the other name: ", pathMesh.format(objInd))
			bpy.ops.import_scene.obj(filepath = pathMesh.format(objInd))
			bpy.context.selected_objects[0].active_material = colors[objInd]
		new_current_objects = [object for object in bpy.context.scene.objects]
		new_objects = set(new_current_objects)-set(prior_objects) 
		pngPath = resultFolder + "/" + nameFileOut + str(frame).zfill(1) + ".png"
		bpy.context.scene.render.filepath = pngPath
		bpy.ops.render.render(write_still = True)
		delete(new_objects)


if(len(argv)<3):
	print("Error!")
	exit()
objectSize = int(argv[0])
recordingNum = int(argv[1])
beforeRecording = int(argv[2])
framesSize = int(recordingNum - beforeRecording)
makesPNGs(firstFrame = 0, lastFrame = framesSize, firstObject=0, lastObject=objectSize)

