from mpi4py import MPI


def get_num_procs():
	return MPI.COMM_WORLD.Get_size()

def get_proc_rank():
	return MPI.COMM_WORLD.Get_rank()

def get_root_proc():
	return 0

def get_root2_proc():
	return 1

def is_root_proc():
	rank = get_proc_rank()
	return rank == get_root_proc()

def is_root2_proc():
	rank = get_proc_rank()
	return rank == get_root2_proc()
	
def broadcast(x, root):
	return MPI.COMM_WORLD.bcast(x, root=root)

def gather(x, root):
	return MPI.COMM_WORLD.gather(x, root=root)

def scatter(x, root):
	return MPI.COMM_WORLD.scatter(x, root=root)

def send(x, dest):
	MPI.COMM_WORLD.send(x, dest=dest, tag=33)

def recv(source):
	return MPI.COMM_WORLD.recv(source=source, tag=33)