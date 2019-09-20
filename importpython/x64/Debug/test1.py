import math
import os

def func(a):
	print(math.fabs(a))
	
	
def GetFileList(str):
	if os.path.exists(str):
		dir = os.listdir(str)
		return tuple(dir)
	else:
		print("The Dir is not exist!")

def func2():
	return ("Hello World",)
		
if __name__ =="__main__":
	pass