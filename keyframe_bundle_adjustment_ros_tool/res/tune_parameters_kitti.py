import os

def main():
	# depth_thres=[0.01,0.1,0.5,1.5,2.0]
	# repr_thres=[0.5,1.0,1.5,2.0,2.5]
	depth_thres=[0.16]
	repr_thres=[1.6]
	weights=[0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0]

	for d in depth_thres:
		for r in repr_thres:
			for w in weights:
				# Run script
				os.system("bash kitti_eval_script.sh %f %f %f"%(d,r,w))

if __name__=="__main__":
	main()